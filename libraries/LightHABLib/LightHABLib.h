/**
 * LightHABLib.h
 *
 * HAB utility library for SAMD21-based APRS Tracker
 *
 * Features:
 *   - Landing position predictor (wind profile + descent prediction)
 *   - Sun/day-night calculations for altitude-aware horizon
 *   - APRS data formatting (lat/lon, timestamps)
 *   - Moving average filter
 *
 * Landing Predictor (all altitudes in feet):
 *   1. During ASCENT through 0-16404ft: Records wind drift (lat/lon rates)
 *      into ~164ft altitude bins using GPS fixes
 *   2. During DESCENT below 16404ft: Uses stored wind profile + current
 *      descent rate to predict landing at multiple target elevations
 *   3. Predictions are transmitted in APRS comment field while still
 *      in coverage, so recovery team has coordinates before signal is lost
 *
 * Memory footprint:
 *   Wind profile: 100 bins x 10 bytes = 1000 bytes
 *   Prediction state: ~80 bytes
 *   Total: ~1.1 KB RAM (landing predictor only)
 *
 * Author: TA2MUN HAB Project
 * Target: SAMD21 (Cortex-M0+, no FPU)
 */

#ifndef LIGHTHAB_LIB_H
#define LIGHTHAB_LIB_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ============================================================
 * Configuration
 * ============================================================ */

#define LP_ALT_BIN_SIZE      164     /* feet per bin (~50m) */
#define LP_MAX_ALT           16404   /* max altitude to track winds (feet, ~5000m) */
#define LP_NUM_BINS          (LP_MAX_ALT / LP_ALT_BIN_SIZE)  /* = 100 */

#define LP_NUM_TARGETS       4       /* prediction target elevations */
#define LP_PREDICT_STEP      164     /* altitude step for prediction (feet, ~50m) */

#define LP_BURST_COUNT       5       /* consecutive descending fixes to detect burst */
#define LP_DESCENT_WINDOW    5       /* number of recent fixes for descent rate calc */
#define LP_MIN_DESCENT_RATE  3.28f   /* minimum descent rate to consider valid (ft/s, ~1 m/s) */
#define LP_MAX_FIX_DT        120.0f  /* max seconds between fixes for wind calc */
                                     /* Works for both 1Hz GPS and APRS intervals */
                                     /* Primary filter is altitude-increasing check */

/* ============================================================
 * Data structures  
 * ============================================================ */

/** Wind data for a single altitude bin */
typedef struct {
    float lat_rate;     /* latitude drift rate (deg/s), running average */
    float lon_rate;     /* longitude drift rate (deg/s), running average */
    uint16_t samples;   /* number of samples accumulated */
} WindBin;              /* 10 bytes per bin */

/** Single GPS fix for descent rate calculation */
typedef struct {
    float lat;          /* degrees */
    float lon;          /* degrees */
    float alt;          /* feet MSL */
    uint32_t time_ms;   /* millis() timestamp */
} GPSFix;

/** Landing prediction for one target elevation */
typedef struct {
    float lat;          /* predicted latitude (degrees) */
    float lon;          /* predicted longitude (degrees) */
    uint16_t target_alt;/* target elevation (feet) */
    bool valid;         /* prediction is valid */
} LandingPrediction;

/** Flight phase enumeration */
typedef enum {
    PHASE_PRE_LAUNCH,   /* on the ground, not yet launched */
    PHASE_ASCENT,       /* ascending - building wind profile */
    PHASE_DESCENT       /* descending - predicting landing */
} FlightPhase;

/** Main predictor state */
typedef struct {
    /* Wind profile from ascent */
    WindBin wind_profile[LP_NUM_BINS];
    
    /* Flight phase tracking */
    FlightPhase phase;
    float max_alt_seen;             /* peak altitude reached (m) */
    uint8_t descent_count;          /* consecutive descending fixes */
    
    /* Recent GPS fixes ring buffer for descent rate */
    GPSFix recent[LP_DESCENT_WINDOW];
    uint8_t recent_idx;             /* next write position */
    uint8_t recent_count;           /* number of valid entries */
    
    /* Previous fix for wind calculation during ascent */
    GPSFix prev_fix;
    bool prev_fix_valid;
    
    /* Target elevations for predictions */
    uint16_t target_alts[LP_NUM_TARGETS];
    
    /* Latest predictions */
    LandingPrediction predictions[LP_NUM_TARGETS];
    
    /* Current measured descent rate (m/s, negative = descending) */
    float current_descent_rate;
    
} LandingPredictor;

/* ============================================================
 * API Functions
 * ============================================================ */

/**
 * Initialize the predictor. Call once at startup.
 * 
 * @param lp        Predictor state
 * @param targets   Array of LP_NUM_TARGETS target elevations in feet
 *                  Example: {13000, 10000, 6500, 3300}
 */
void lp_init(LandingPredictor *lp, const uint16_t targets[LP_NUM_TARGETS]);

/**
 * Feed a new GPS fix to the predictor. Call on every GPS update (1 Hz).
 * 
 * During ascent: updates wind profile for altitudes 0-16404ft
 * During descent below 16404ft: recalculates landing predictions
 *
 * @param lp        Predictor state
 * @param lat       Latitude (degrees, positive = N)
 * @param lon       Longitude (degrees, positive = E)
 * @param alt_ft    Altitude MSL (feet)
 * @param time_ms   Current millis() timestamp
 */
void lp_update(LandingPredictor *lp, float lat, float lon,
               float alt_ft, uint32_t time_ms);

/**
 * Check if predictions are available and fresh.
 * 
 * @param lp        Predictor state
 * @return          true if in descent phase with valid predictions
 */
bool lp_has_predictions(const LandingPredictor *lp);

/**
 * Get prediction for a specific target index (0 to LP_NUM_TARGETS-1).
 * 
 * @param lp        Predictor state
 * @param idx       Target index
 * @return          Pointer to prediction (check ->valid before use)
 */
const LandingPrediction* lp_get_prediction(const LandingPredictor *lp, uint8_t idx);

/**
 * Get current descent rate (ft/s, negative = descending).
 * 
 * @param lp        Predictor state
 * @return          Descent rate, or 0 if not in descent
 */
float lp_get_descent_rate(const LandingPredictor *lp);

/**
 * Format prediction into APRS comment string.
 * 
 * Output example: "L13000:39.0750,30.4475 V-16.4"
 * If multiple predictions: "L13000:39.08,30.45 L10000:39.07,30.44 L6500:39.06,30.43"
 * 
 * @param lp        Predictor state
 * @param buf       Output buffer
 * @param buf_len   Buffer size
 * @return          Number of characters written
 */
int lp_format_aprs(const LandingPredictor *lp, char *buf, int buf_len);

/**
 * Get number of wind bins populated during ascent.
 * Useful for diagnostics.
 *
 * @param lp        Predictor state
 * @return          Number of bins with at least 1 sample
 */
int lp_wind_bins_populated(const LandingPredictor *lp);

/* ============================================================
 * Sun / Day-Night Calculations
 * ============================================================ */

/** Location and time for solar calculations */
typedef struct {
    double latitude;
    double longitude;
    double altitude;    /* meters MSL */
    int hour;           /* UTC hour */
    int minute;         /* UTC minute */
    int dayOfYear;      /* 1-366 */
} LocationTime;

/** Day/night analysis result */
typedef struct {
    bool isDay;
    double sunElevation;    /* degrees above horizon */
    double timeToSunset;    /* hours (0 if not calculated) */
    double timeToSunrise;   /* hours (0 if not calculated) */
} DayNightStatus;

/**
 * Calculate geometric horizon depression due to altitude.
 *
 * @param altitude_meters   Observer altitude MSL (meters)
 * @return                  Horizon depression in degrees
 */
double hab_calculateHorizonDepression(double altitude_meters);

/**
 * Calculate atmospheric refraction correction.
 *
 * @param altitude_meters       Observer altitude MSL (meters)
 * @param apparent_elevation_deg Apparent sun elevation (degrees)
 * @return                      Refraction correction in degrees
 */
double hab_calculateAtmosphericRefraction(double altitude_meters, double apparent_elevation_deg);

/**
 * Full day/night analysis with sun elevation, refraction, and horizon depression.
 *
 * @param loc   Location and time
 * @return      Day/night status
 */
DayNightStatus hab_analyzeDayNight(LocationTime loc);

/**
 * Simplified day/night check for a given position and time.
 *
 * @return true if daytime
 */
bool hab_isDayTimeWithAltitude(double lat, double lon, double altitude,
                               int hour, int minute, int dayOfYear);

/**
 * Calculate day of year from date.
 *
 * @return Day of year (1-366)
 */
int hab_calculateDayOfYear(int year, int month, int day);

/* ============================================================
 * APRS Data Formatting
 * ============================================================ */

/**
 * Encode HMS timestamp for APRS (format: "HHMMSSh").
 * Output buffer must be at least 8 bytes.
 *
 * @param hour    UTC hour
 * @param minute  UTC minute
 * @param second  UTC second
 * @param out     Output buffer (minimum 8 bytes)
 * @param outLen  Buffer size
 */
void hab_encodeHMSTimestamp(int hour, int minute, int second, char *out, size_t outLen);

/**
 * Format latitude for APRS (format: "ddmm.hhN/S").
 * Output buffer must be at least 9 bytes.
 *
 * @param latitude  Latitude in degrees (positive = N)
 * @param out       Output buffer (minimum 9 bytes)
 * @param outLen    Buffer size
 */
void hab_createLatForAPRS(double latitude, char *out, size_t outLen);

/**
 * Format longitude for APRS (format: "dddmm.hhE/W").
 * Output buffer must be at least 10 bytes.
 *
 * @param longitude Longitude in degrees (positive = E)
 * @param out       Output buffer (minimum 10 bytes)
 * @param outLen    Buffer size
 */
void hab_createLongForAPRS(double longitude, char *out, size_t outLen);

/* ============================================================
 * Moving Average Filter
 * ============================================================ */

/** Circular buffer moving average state */
typedef struct {
    float *samples;       /* caller-provided buffer */
    uint16_t capacity;    /* buffer size */
    uint16_t index;       /* current write position */
    bool bufferFull;      /* has wrapped around at least once */
    float average;        /* current computed average */
    float maxAverage;     /* peak average ever seen */
} MovingAverage;

/**
 * Initialize a moving average filter.
 *
 * @param ma        Moving average state
 * @param buffer    Caller-provided sample buffer
 * @param capacity  Buffer size (number of samples)
 */
void hab_ma_init(MovingAverage *ma, float *buffer, uint16_t capacity);

/**
 * Add a new sample and recompute the average.
 *
 * @param ma    Moving average state
 * @param value New sample value
 */
void hab_ma_update(MovingAverage *ma, float value);

/* ============================================================
 * GPS Flight Simulator
 * ============================================================ */

/** Simulation mode */
typedef enum {
    SIM_MODE_WIND_LAYERS = 0,   /* Physics-based with configurable wind layers */
    SIM_MODE_WAYPOINTS   = 1    /* Replay pre-computed trajectory (e.g. sondehub) */
} SimMode;

/** Simulation flight phase */
typedef enum {
    SIM_PHASE_PRE_LAUNCH = 0, /* waiting at launch site (ascent delay) */
    SIM_PHASE_ASCENT,
    SIM_PHASE_DESCENT,
    SIM_PHASE_LANDED
} SimFlightPhase;

/** Wind layer: constant wind in an altitude band */
typedef struct {
    float minAlt;           /* feet, lower bound (inclusive) */
    float maxAlt;           /* feet, upper bound (exclusive) */
    float windSpeed;        /* knots */
    float windDirection;    /* degrees, meteorological (direction FROM, 0=N, 90=E) */
} SimWindLayer;

/** Waypoint for trajectory replay */
typedef struct {
    double lat;             /* degrees */
    double lon;             /* degrees */
    float alt;              /* feet MSL */
    uint32_t timeOffsetMs;  /* milliseconds from simulation start */
} SimWaypoint;

/** Simulator configuration (set once before init) */
typedef struct {
    SimMode mode;

    /* Launch site */
    double launchLat;       /* degrees */
    double launchLon;       /* degrees */
    float launchAlt;        /* feet MSL */

    /* Flight parameters (wind layer mode) */
    uint32_t ascentDelayMs; /* ms to stay at launch position before ascending */
    float ascentRate;       /* ft/s, positive */
    float descentRate;      /* ft/s, positive (terminal velocity near ground) */
    float burstAltitude;    /* feet MSL */

    /* Simulated GPS */
    uint8_t satellites;     /* simulated satellite count */
    bool useGPSTime;        /* true = use real GPS clock, false = simulate */
    uint8_t startHour;      /* UTC start hour (if !useGPSTime) */
    uint8_t startMinute;    /* UTC start minute */
    uint8_t startSecond;    /* UTC start second */
    uint16_t startDayOfYear;/* day of year for isNight calc (1-366) */

    /* Wind layer mode data */
    const SimWindLayer *windLayers;
    uint8_t windLayerCount;

    /* Waypoint mode data */
    const SimWaypoint *waypoints;
    uint16_t waypointCount;
} SimConfig;

/** Simulator state (mutable, updated each tick) */
typedef struct {
    double lat;             /* degrees */
    double lon;             /* degrees */
    float altitude;         /* feet MSL */
    float speed;            /* knots (ground speed) */
    float course;           /* degrees, 0-360 */
    uint8_t satellites;
    uint8_t hour;           /* UTC */
    uint8_t minute;
    uint8_t second;
    uint16_t dayOfYear;     /* 1-366 */
    bool fix;               /* simulated GPS fix (true once started) */
    bool landed;
    SimFlightPhase phase;
    uint32_t elapsedMs;     /* total simulated time */
    uint16_t _wpIndex;      /* internal: current waypoint index for interpolation */
} SimState;

/**
 * Initialize the simulator. Call once at startup.
 *
 * @param state   Simulator state to initialize
 * @param config  Simulator configuration
 */
void hab_sim_init(SimState *state, const SimConfig *config);

/**
 * Advance the simulation by one time step.
 *
 * @param state   Simulator state
 * @param config  Simulator configuration
 * @param stepMs  Time step in milliseconds (e.g. 15000 for 15s)
 */
void hab_sim_update(SimState *state, const SimConfig *config, uint32_t stepMs);

/* ============================================================
 * Math Utilities
 * ============================================================ */

/**
 * Generate a random float in [minVal, maxVal].
 *
 * @param minVal  Minimum value
 * @param maxVal  Maximum value
 * @return        Random float in range
 */
float hab_randomFloat(float minVal, float maxVal);

#endif /* LIGHTHAB_LIB_H */
