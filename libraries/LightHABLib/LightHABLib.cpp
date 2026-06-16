/**
 * LightHABLib.cpp
 *
 * HAB utility library implementation
 * Target: SAMD21 (Cortex-M0+, no FPU, software float)
 */

#include <Arduino.h>
#include "LightHABLib.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/dtostrf.h>

#ifndef PI
#define PI 3.14159265358979323846
#endif

/* ============================================================
 * Internal helpers
 * ============================================================ */

/** Get bin index for an altitude. Returns -1 if out of range. */
static int get_bin(float alt_m) {
    if (alt_m < 0 || alt_m >= LP_MAX_ALT) return -1;
    return (int)(alt_m / LP_ALT_BIN_SIZE);
}

/**
 * Estimate descent rate at a different altitude using air density scaling.
 * 
 * Terminal velocity scales as 1/sqrt(rho).
 * Using barometric approximation: rho ~ rho0 * exp(-h / H)
 * So: v(h) = v_ref * exp((h - h_ref) / (2*H))
 * 
 * We use 2*H = 55774ft (scale height H ≈ 27887ft, equivalent to ~8500m)
 */
static float scale_descent_rate(float rate_ref, float alt_ref, float alt_target) {
    float exponent = (alt_target - alt_ref) / 55774.0f;

    /* Fast exp approximation for small exponents (|x| < 1)
     * Good enough for our altitude range of 0-16404ft delta
     * exp(x) ≈ 1 + x + x²/2 + x³/6
     * Max error ~0.2% for |x| < 0.3 (which is 16404ft / 55774)
     */
    float x = exponent;
    float exp_approx = 1.0f + x + (x * x) * 0.5f + (x * x * x) * 0.1667f;
    
    return rate_ref * exp_approx;
}

/**
 * Interpolate wind at a given altitude from the wind profile.
 * Uses linear interpolation between neighboring bins.
 */
static void get_wind_at_alt(const LandingPredictor *lp, float alt_m,
                             float *lat_rate, float *lon_rate) {
    int bin = get_bin(alt_m);
    
    /* Default: no wind */
    *lat_rate = 0.0f;
    *lon_rate = 0.0f;
    
    if (bin < 0) return;
    
    /* If this bin has data, use it */
    if (lp->wind_profile[bin].samples > 0) {
        *lat_rate = lp->wind_profile[bin].lat_rate;
        *lon_rate = lp->wind_profile[bin].lon_rate;
        
        /* Try to interpolate with neighbor for smoother result */
        float bin_center = (bin + 0.5f) * LP_ALT_BIN_SIZE;
        int neighbor;
        
        if (alt_m > bin_center && bin + 1 < LP_NUM_BINS) {
            neighbor = bin + 1;
        } else if (alt_m < bin_center && bin > 0) {
            neighbor = bin - 1;
        } else {
            return; /* no interpolation needed */
        }
        
        if (lp->wind_profile[neighbor].samples > 0) {
            float neighbor_center = (neighbor + 0.5f) * LP_ALT_BIN_SIZE;
            float frac = (alt_m - bin_center) / (neighbor_center - bin_center);
            
            *lat_rate = lp->wind_profile[bin].lat_rate + 
                        frac * (lp->wind_profile[neighbor].lat_rate - 
                                lp->wind_profile[bin].lat_rate);
            *lon_rate = lp->wind_profile[bin].lon_rate + 
                        frac * (lp->wind_profile[neighbor].lon_rate - 
                                lp->wind_profile[bin].lon_rate);
        }
        return;
    }
    
    /* Bin empty - search for nearest populated bins above and below */
    int below = -1, above = -1;
    for (int i = bin - 1; i >= 0; i--) {
        if (lp->wind_profile[i].samples > 0) { below = i; break; }
    }
    for (int i = bin + 1; i < LP_NUM_BINS; i++) {
        if (lp->wind_profile[i].samples > 0) { above = i; break; }
    }
    
    if (below >= 0 && above >= 0) {
        /* Interpolate between nearest populated bins */
        float below_center = (below + 0.5f) * LP_ALT_BIN_SIZE;
        float above_center = (above + 0.5f) * LP_ALT_BIN_SIZE;
        float frac = (alt_m - below_center) / (above_center - below_center);
        
        *lat_rate = lp->wind_profile[below].lat_rate + 
                    frac * (lp->wind_profile[above].lat_rate - 
                            lp->wind_profile[below].lat_rate);
        *lon_rate = lp->wind_profile[below].lon_rate + 
                    frac * (lp->wind_profile[above].lon_rate - 
                            lp->wind_profile[below].lon_rate);
    } else if (below >= 0) {
        *lat_rate = lp->wind_profile[below].lat_rate;
        *lon_rate = lp->wind_profile[below].lon_rate;
    } else if (above >= 0) {
        *lat_rate = lp->wind_profile[above].lat_rate;
        *lon_rate = lp->wind_profile[above].lon_rate;
    }
}

/** 
 * Calculate descent rate from recent fixes ring buffer.
 * Uses the oldest and newest valid fixes for a stable rate.
 */
static float calc_descent_rate(const LandingPredictor *lp) {
    if (lp->recent_count < 2) return 0.0f;
    
    /* Find oldest and newest in ring buffer */
    int oldest_idx, newest_idx;
    
    if (lp->recent_count >= LP_DESCENT_WINDOW) {
        oldest_idx = lp->recent_idx; /* next write pos = oldest */
        newest_idx = (lp->recent_idx + LP_DESCENT_WINDOW - 1) % LP_DESCENT_WINDOW;
    } else {
        oldest_idx = 0;
        newest_idx = lp->recent_count - 1;
    }
    
    const GPSFix *oldest = &lp->recent[oldest_idx];
    const GPSFix *newest = &lp->recent[newest_idx];
    
    float dt = (float)(newest->time_ms - oldest->time_ms) / 1000.0f;
    if (dt < 0.5f) return 0.0f;
    
    return (newest->alt - oldest->alt) / dt;
}

/**
 * Run landing prediction for all target elevations.
 * Steps down from current position using wind profile and descent rate.
 */
static void run_predictions(LandingPredictor *lp, 
                             float cur_lat, float cur_lon, float cur_alt) {
    float descent_rate = lp->current_descent_rate;
    
    /* Need a meaningful descent rate to predict */
    if (descent_rate > -LP_MIN_DESCENT_RATE) {
        for (int t = 0; t < LP_NUM_TARGETS; t++) {
            lp->predictions[t].valid = false;
        }
        return;
    }
    
    /* For each target elevation, step down from current position */
    for (int t = 0; t < LP_NUM_TARGETS; t++) {
        float target = (float)lp->target_alts[t];
        
        /* Skip if we're already below this target */
        if (cur_alt <= target) {
            lp->predictions[t].valid = false;
            continue;
        }
        
        float pred_lat = cur_lat;
        float pred_lon = cur_lon;
        float pred_alt = cur_alt;
        float ref_alt = cur_alt;
        float ref_rate = descent_rate; /* negative value */
        
        while (pred_alt > target) {
            /* Get wind drift at this altitude from ascent profile */
            float wind_lat, wind_lon;
            get_wind_at_alt(lp, pred_alt, &wind_lat, &wind_lon);
            
            /* Scale descent rate for current altitude */
            float v_desc = scale_descent_rate(ref_rate, ref_alt, pred_alt);
            if (v_desc > -LP_MIN_DESCENT_RATE) v_desc = -LP_MIN_DESCENT_RATE;
            
            /* Time to descend one step */
            float alt_step = pred_alt - target;
            if (alt_step > LP_PREDICT_STEP) alt_step = LP_PREDICT_STEP;
            
            float dt = alt_step / (-v_desc); /* v_desc is negative */
            
            /* Apply wind drift */
            pred_lat += wind_lat * dt;
            pred_lon += wind_lon * dt;
            pred_alt -= alt_step;
        }
        
        lp->predictions[t].lat = pred_lat;
        lp->predictions[t].lon = pred_lon;
        lp->predictions[t].target_alt = lp->target_alts[t];
        lp->predictions[t].valid = true;
    }
}

/* ============================================================
 * Public API Implementation
 * ============================================================ */

void lp_init(LandingPredictor *lp, const uint16_t targets[LP_NUM_TARGETS]) {
    memset(lp, 0, sizeof(LandingPredictor));
    lp->phase = PHASE_PRE_LAUNCH;
    
    for (int i = 0; i < LP_NUM_TARGETS; i++) {
        lp->target_alts[i] = targets[i];
    }
}

void lp_update(LandingPredictor *lp, float lat, float lon,
               float alt_ft, uint32_t time_ms) {

    /* ---- Input validation ---- */
    if (isnan(lat) || isnan(lon) || isnan(alt_ft) || isinf(alt_ft)) return;

    /* ---- Track peak altitude ---- */
    if (alt_ft > lp->max_alt_seen) {
        lp->max_alt_seen = alt_ft;
        lp->descent_count = 0;
    } else {
        lp->descent_count++;
    }

    /* ---- Phase detection ---- */
    switch (lp->phase) {
        case PHASE_PRE_LAUNCH:
            /* Transition to ascent when we've gained significant altitude (1640ft ~ 500m) */
            if (alt_ft > 1640.0f && lp->max_alt_seen > 1640.0f) {
                lp->phase = PHASE_ASCENT;
            }
            break;

        case PHASE_ASCENT:
            /* Transition to descent after LP_BURST_COUNT consecutive
             * descending fixes AND we've been above 16404ft at some point */
            if (lp->descent_count >= LP_BURST_COUNT &&
                lp->max_alt_seen > LP_MAX_ALT) {
                lp->phase = PHASE_DESCENT;
            }
            break;

        case PHASE_DESCENT:
            /* Stay in descent until landing */
            break;
    }

    /* ---- Ascent: build wind profile for 0-16404ft ---- */
    if (lp->phase == PHASE_ASCENT && alt_ft < LP_MAX_ALT) {
        if (lp->prev_fix_valid) {
            float dt = (float)(time_ms - lp->prev_fix.time_ms) / 1000.0f;

            /* Sanity check: 0.5s < dt < LP_MAX_FIX_DT, altitude increasing */
            if (dt > 0.5f && dt < LP_MAX_FIX_DT && alt_ft > lp->prev_fix.alt) {
                float lat_rate = (lat - lp->prev_fix.lat) / dt;
                float lon_rate = (lon - lp->prev_fix.lon) / dt;

                /* Store in bin at midpoint altitude */
                float mid_alt = (lp->prev_fix.alt + alt_ft) / 2.0f;
                int bin = get_bin(mid_alt);

                if (bin >= 0 && bin < LP_NUM_BINS) {
                    /* Running average */
                    uint16_t n = lp->wind_profile[bin].samples;
                    lp->wind_profile[bin].lat_rate =
                        (lp->wind_profile[bin].lat_rate * n + lat_rate) / (n + 1);
                    lp->wind_profile[bin].lon_rate =
                        (lp->wind_profile[bin].lon_rate * n + lon_rate) / (n + 1);
                    lp->wind_profile[bin].samples = n + 1;
                }
            }
        }

        /* Save current fix as previous */
        lp->prev_fix.lat = lat;
        lp->prev_fix.lon = lon;
        lp->prev_fix.alt = alt_ft;
        lp->prev_fix.time_ms = time_ms;
        lp->prev_fix_valid = true;
    }

    /* ---- Descent: calculate descent rate & predict landing ---- */
    if (lp->phase == PHASE_DESCENT) {
        /* Add to recent fixes ring buffer */
        lp->recent[lp->recent_idx].lat = lat;
        lp->recent[lp->recent_idx].lon = lon;
        lp->recent[lp->recent_idx].alt = alt_ft;
        lp->recent[lp->recent_idx].time_ms = time_ms;
        lp->recent_idx = (lp->recent_idx + 1) % LP_DESCENT_WINDOW;
        if (lp->recent_count < LP_DESCENT_WINDOW) lp->recent_count++;

        /* Calculate current descent rate from recent fixes (ft/s) */
        lp->current_descent_rate = calc_descent_rate(lp);

        /* Run predictions when below 16404ft and have wind data */
        if (alt_ft < LP_MAX_ALT && lp->current_descent_rate < -LP_MIN_DESCENT_RATE) {
            run_predictions(lp, lat, lon, alt_ft);
        }
    }
}

bool lp_has_predictions(const LandingPredictor *lp) {
    if (lp->phase != PHASE_DESCENT) return false;
    for (int i = 0; i < LP_NUM_TARGETS; i++) {
        if (lp->predictions[i].valid) return true;
    }
    return false;
}

const LandingPrediction* lp_get_prediction(const LandingPredictor *lp, uint8_t idx) {
    if (idx >= LP_NUM_TARGETS) return NULL;
    return &lp->predictions[idx];
}

float lp_get_descent_rate(const LandingPredictor *lp) {
    return lp->current_descent_rate;
}

int lp_format_aprs(const LandingPredictor *lp, char *buf, int buf_len) {
    if (!lp_has_predictions(lp)) return 0;
    
    int written = 0;
    
    for (int i = 0; i < LP_NUM_TARGETS; i++) {
        if (!lp->predictions[i].valid) continue;
        
        int target_ft = lp->predictions[i].target_alt;
        int n;

        /* Format: L6500:39.0750,30.4475 (altitude in feet) */
        n = snprintf(buf + written, buf_len - written,
                     "%sL%d:%.4f,%.4f",
                     (written > 0) ? " " : "",
                     target_ft,
                     lp->predictions[i].lat,
                     lp->predictions[i].lon);
        
        if (n > 0 && written + n < buf_len) {
            written += n;
        } else {
            break;
        }
    }
    
    /* Append descent rate (ft/s) */
    if (written > 0 && written + 8 < buf_len) {
        int n = snprintf(buf + written, buf_len - written,
                         " V%.1f", lp->current_descent_rate);
        if (n > 0) written += n;
    }
    
    return written;
}

int lp_wind_bins_populated(const LandingPredictor *lp) {
    int count = 0;
    for (int i = 0; i < LP_NUM_BINS; i++) {
        if (lp->wind_profile[i].samples > 0) count++;
    }
    return count;
}

/* ============================================================
 * Sun / Day-Night Calculations
 * ============================================================ */

double hab_calculateHorizonDepression(double altitude_meters) {
    double earth_radius = 6371000.0;
    if (altitude_meters <= 0) return 0;
    double geometric_horizon = acos(earth_radius / (earth_radius + altitude_meters));
    return geometric_horizon * 180.0 / PI;
}

double hab_calculateAtmosphericRefraction(double altitude_meters, double apparent_elevation_deg) {
    double p0 = 1013.25; /* hPa */
    double pressure = p0 * pow(1 - (0.0065 * altitude_meters / 288.15), 5.255);
    double pressure_ratio = pressure / p0;

    if (apparent_elevation_deg < -5) return 0; /* avoid calculation errors */
    double refraction = 0.0167 * pressure_ratio /
        tan((apparent_elevation_deg + 7.31 / (apparent_elevation_deg + 4.4)) * PI / 180);
    return refraction;
}

DayNightStatus hab_analyzeDayNight(LocationTime loc) {
    DayNightStatus status;

    double latRad = loc.latitude * PI / 180;
    double declination = 23.45 * sin((360.0 / 365.0) * (loc.dayOfYear - 81) * PI / 180);
    double declinationRad = declination * PI / 180;

    double timeDecimal = loc.hour + loc.minute / 60.0;
    double hourAngle = 15.0 * (timeDecimal - 12.0) * PI / 180;

    double elevation = asin(sin(latRad) * sin(declinationRad) +
                            cos(latRad) * cos(declinationRad) * cos(hourAngle));

    double elevationDeg = elevation * 180 / PI;
    double refraction = hab_calculateAtmosphericRefraction(loc.altitude, elevationDeg);
    double correctedElevation = elevationDeg + refraction;

    double horizonDepression = hab_calculateHorizonDepression(loc.altitude);
    double effectiveHorizon = -0.833 - horizonDepression;

    status.sunElevation = correctedElevation;
    status.isDay = correctedElevation > effectiveHorizon;
    status.timeToSunset = 0;
    status.timeToSunrise = 0;

    return status;
}

bool hab_isDayTimeWithAltitude(double lat, double lon, double altitude,
                               int hour, int minute, int dayOfYear) {
    LocationTime loc = {lat, lon, altitude, hour, minute, dayOfYear};
    DayNightStatus status = hab_analyzeDayNight(loc);
    return status.isDay;
}

int hab_calculateDayOfYear(int year, int month, int day) {
    int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
        daysInMonth[1] = 29;
    }

    int dayOfYear = 0;
    for (int i = 0; i < month - 1; i++) {
        dayOfYear += daysInMonth[i];
    }
    dayOfYear += day;

    return dayOfYear;
}

/* ============================================================
 * APRS Data Formatting
 * ============================================================ */

void hab_encodeHMSTimestamp(int hour, int minute, int second, char *out, size_t outLen) {
    if (outLen < 8) return;
    sprintf(out, "%02d%02d%02d", hour, minute, second);
    out[6] = 'h';
    out[7] = '\0';
}

void hab_createLatForAPRS(double latitude, char *out, size_t outLen) {
    if (outLen < 9) return;

    char lat_buff[10];
    int temp = 0;
    double dm_lat = 0.0;

    if (latitude < 0.0) {
        temp = -(int)latitude;
        dm_lat = temp * 100.0 - (latitude + temp) * 60.0;
    } else {
        temp = (int)latitude;
        dm_lat = temp * 100 + (latitude - temp) * 60.0;
    }

    dtostrf(dm_lat, 7, 2, lat_buff);

    if (dm_lat < 1000) {
        lat_buff[0] = '0';
    }

    lat_buff[7] = (latitude >= 0.0) ? 'N' : 'S';
    lat_buff[8] = '\0';

    memcpy(out, lat_buff, 9);
}

void hab_createLongForAPRS(double longitude, char *out, size_t outLen) {
    if (outLen < 10) return;

    char long_buff[11];
    int temp = 0;
    double dm_lon = 0.0;

    if (longitude < 0.0) {
        temp = -(int)longitude;
        dm_lon = temp * 100.0 - (longitude + temp) * 60.0;
    } else {
        temp = (int)longitude;
        dm_lon = temp * 100 + (longitude - temp) * 60.0;
    }

    dtostrf(dm_lon, 8, 2, long_buff);

    if (dm_lon < 10000) {
        long_buff[0] = '0';
    }
    if (dm_lon < 1000) {
        long_buff[1] = '0';
    }

    long_buff[8] = (longitude >= 0.0) ? 'E' : 'W';
    long_buff[9] = '\0';

    memcpy(out, long_buff, 10);
}

/* ============================================================
 * Moving Average Filter
 * ============================================================ */

void hab_ma_init(MovingAverage *ma, float *buffer, uint16_t capacity) {
    ma->samples = buffer;
    ma->capacity = capacity;
    ma->index = 0;
    ma->bufferFull = false;
    ma->average = 0.0f;
    ma->maxAverage = 0.0f;

    for (uint16_t i = 0; i < capacity; i++) {
        buffer[i] = 0.0f;
    }
}

void hab_ma_update(MovingAverage *ma, float value) {
    ma->samples[ma->index] = value;
    ma->index = (ma->index + 1) % ma->capacity;

    if (ma->index == 0) {
        ma->bufferFull = true;
    }

    float sum = 0.0f;
    uint16_t count = ma->bufferFull ? ma->capacity : ma->index;

    for (uint16_t i = 0; i < count; i++) {
        sum += ma->samples[i];
    }

    ma->average = sum / count;

    if (ma->average > ma->maxAverage) {
        ma->maxAverage = ma->average;
    }
}

/* ============================================================
 * Math Utilities
 * ============================================================ */

float hab_randomFloat(float minVal, float maxVal) {
    if (minVal >= maxVal) {
        return minVal;
    }
    float randomValue = (float)(random(10000)) / 10000.0f;
    return minVal + (randomValue * (maxVal - minVal));
}

/* ============================================================
 * GPS Flight Simulator
 * ============================================================ */

#define SIM_KNOTS_TO_MS     0.514444f
#define SIM_MS_TO_KNOTS     (1.0f / SIM_KNOTS_TO_MS)
#define SIM_METERS_PER_DEG_LAT  111320.0

/** Meters per degree of longitude at a given latitude */
static double sim_meters_per_deg_lon(double lat) {
    return 111320.0 * cos(lat * PI / 180.0);
}

/**
 * Convert wind speed (knots) and direction (meteorological FROM)
 * into lat/lon drift rates (degrees/second).
 */
static void sim_wind_to_drift(float windSpeed, float windDirection,
                               double lat,
                               float *dLatPerSec, float *dLonPerSec) {
    /* Wind direction is "from", movement is opposite */
    float movDirRad = (windDirection + 180.0f) * (float)(PI / 180.0);
    float speedMs = windSpeed * SIM_KNOTS_TO_MS;

    float dNorth = speedMs * cosf(movDirRad);   /* m/s northward */
    float dEast  = speedMs * sinf(movDirRad);   /* m/s eastward */

    *dLatPerSec = (float)(dNorth / SIM_METERS_PER_DEG_LAT);

    double mPerDegLon = sim_meters_per_deg_lon(lat);
    if (mPerDegLon < 1.0) mPerDegLon = 1.0; /* safety near poles */
    *dLonPerSec = (float)(dEast / mPerDegLon);
}

/**
 * Get wind drift rates at a given altitude from the wind layer config.
 * Uses the matching layer, or nearest layer if altitude is outside all ranges.
 */
static void sim_get_wind(const SimConfig *config, float alt, double lat,
                          float *dLatPerSec, float *dLonPerSec) {
    *dLatPerSec = 0.0f;
    *dLonPerSec = 0.0f;

    if (config->windLayerCount == 0 || config->windLayers == NULL) return;

    /* Search for matching layer */
    for (uint8_t i = 0; i < config->windLayerCount; i++) {
        if (alt >= config->windLayers[i].minAlt && alt < config->windLayers[i].maxAlt) {
            sim_wind_to_drift(config->windLayers[i].windSpeed,
                              config->windLayers[i].windDirection,
                              lat, dLatPerSec, dLonPerSec);
            return;
        }
    }

    /* Not in any layer - use nearest (lowest or highest) */
    if (alt < config->windLayers[0].minAlt) {
        sim_wind_to_drift(config->windLayers[0].windSpeed,
                          config->windLayers[0].windDirection,
                          lat, dLatPerSec, dLonPerSec);
    } else {
        uint8_t last = config->windLayerCount - 1;
        sim_wind_to_drift(config->windLayers[last].windSpeed,
                          config->windLayers[last].windDirection,
                          lat, dLatPerSec, dLonPerSec);
    }
}

/** Advance simulated clock by given milliseconds */
static void sim_advance_time(SimState *state, uint32_t ms) {
    uint32_t totalSec = ms / 1000;
    uint32_t newSec = state->second + totalSec;

    state->second = newSec % 60;
    uint32_t newMin = state->minute + newSec / 60;
    state->minute = newMin % 60;
    uint32_t newHour = state->hour + newMin / 60;
    state->hour = newHour % 24;

    /* Advance day of year if hours wrapped */
    if (newHour >= 24) {
        state->dayOfYear += (uint16_t)(newHour / 24);
        if (state->dayOfYear > 366) state->dayOfYear = 1;
    }
}

/**
 * Calculate course (degrees 0-360) and speed (knots) from position delta.
 */
static void sim_calc_course_speed(double prevLat, double prevLon,
                                   double newLat, double newLon,
                                   float dtSec,
                                   float *course, float *speed) {
    double dLatM = (newLat - prevLat) * SIM_METERS_PER_DEG_LAT;
    double dLonM = (newLon - prevLon) * sim_meters_per_deg_lon(newLat);
    double distM = sqrt(dLatM * dLatM + dLonM * dLonM);

    *speed = (float)(distM / dtSec) * SIM_MS_TO_KNOTS;

    if (distM > 0.01) {
        double crs = atan2(dLonM, dLatM) * 180.0 / PI;
        if (crs < 0) crs += 360.0;
        *course = (float)crs;
    }
    /* else keep previous course */
}

/* ---- Wind Layer Mode update ---- */
static void sim_update_wind_layers(SimState *state, const SimConfig *config,
                                    float dtSec) {
    /* Save previous position for course/speed */
    double prevLat = state->lat;
    double prevLon = state->lon;

    /* Pre-launch: stay at launch position until delay expires */
    if (state->phase == SIM_PHASE_PRE_LAUNCH) {
        if (state->elapsedMs >= config->ascentDelayMs) {
            state->phase = SIM_PHASE_ASCENT;
        } else {
            return; /* no movement during pre-launch */
        }
    }

    /* Altitude update */
    if (state->phase == SIM_PHASE_ASCENT) {
        state->altitude += config->ascentRate * dtSec;
        if (state->altitude >= config->burstAltitude) {
            state->altitude = config->burstAltitude;
            state->phase = SIM_PHASE_DESCENT;
        }
    } else if (state->phase == SIM_PHASE_DESCENT) {
        /* Scale descent rate with altitude (air density effect) */
        /* v(h) = v_ground * exp(h / (2*H)), H~27887ft, so 2H=55774ft */
        float exponent = state->altitude / 55774.0f;
        float x = exponent;
        float scale = 1.0f + x + (x * x) * 0.5f + (x * x * x) * 0.1667f;
        float currentDescentRate = config->descentRate * scale;

        state->altitude -= currentDescentRate * dtSec;
        if (state->altitude <= config->launchAlt) {
            state->altitude = config->launchAlt;
            state->phase = SIM_PHASE_LANDED;
            state->landed = true;
            state->speed = 0;
        }
    }

    /* Horizontal wind drift */
    float dLat, dLon;
    sim_get_wind(config, state->altitude, state->lat, &dLat, &dLon);
    state->lat += dLat * dtSec;
    state->lon += dLon * dtSec;

    /* Course and speed */
    if (!state->landed && dtSec > 0) {
        sim_calc_course_speed(prevLat, prevLon, state->lat, state->lon,
                              dtSec, &state->course, &state->speed);
    }
}

/* ---- Waypoint Mode update ---- */
static void sim_update_waypoints(SimState *state, const SimConfig *config) {
    if (config->waypointCount == 0 || config->waypoints == NULL) return;

    uint32_t t = state->elapsedMs;
    uint16_t last = config->waypointCount - 1;

    /* Past the last waypoint - landed */
    if (t >= config->waypoints[last].timeOffsetMs) {
        double prevLat = state->lat;
        double prevLon = state->lon;
        state->lat = config->waypoints[last].lat;
        state->lon = config->waypoints[last].lon;
        state->altitude = config->waypoints[last].alt;
        state->phase = SIM_PHASE_LANDED;
        state->landed = true;
        state->speed = 0;
        state->course = 0;
        return;
    }

    /* Find bracketing waypoints (scan forward from last known index) */
    uint16_t idx = state->_wpIndex;
    while (idx < last && config->waypoints[idx + 1].timeOffsetMs <= t) {
        idx++;
    }
    state->_wpIndex = idx;

    /* Interpolate between waypoints[idx] and waypoints[idx+1] */
    const SimWaypoint *wp0 = &config->waypoints[idx];
    const SimWaypoint *wp1 = &config->waypoints[idx + 1];

    uint32_t segDuration = wp1->timeOffsetMs - wp0->timeOffsetMs;
    float frac = (segDuration > 0) ? (float)(t - wp0->timeOffsetMs) / (float)segDuration : 0.0f;

    double prevLat = state->lat;
    double prevLon = state->lon;
    float prevAlt = state->altitude;

    state->lat = wp0->lat + frac * (wp1->lat - wp0->lat);
    state->lon = wp0->lon + frac * (wp1->lon - wp0->lon);
    state->altitude = wp0->alt + frac * (wp1->alt - wp0->alt);

    /* Determine phase from altitude change */
    if (state->altitude > prevAlt + 0.1f) {
        state->phase = SIM_PHASE_ASCENT;
    } else if (state->altitude < prevAlt - 0.1f) {
        state->phase = SIM_PHASE_DESCENT;
    }

    /* Course and speed from position delta */
    if (state->elapsedMs > 0) {
        /* Use waypoint segment to calculate speed/course (more stable) */
        float segSec = segDuration / 1000.0f;
        if (segSec > 0) {
            sim_calc_course_speed(wp0->lat, wp0->lon, wp1->lat, wp1->lon,
                                  segSec, &state->course, &state->speed);
        }
    }
}

/* ---- Public API ---- */

void hab_sim_init(SimState *state, const SimConfig *config) {
    memset(state, 0, sizeof(SimState));
    state->lat = config->launchLat;
    state->lon = config->launchLon;
    state->altitude = config->launchAlt;
    state->satellites = config->satellites;
    state->fix = true;
    state->landed = false;
    state->phase = (config->ascentDelayMs > 0) ? SIM_PHASE_PRE_LAUNCH : SIM_PHASE_ASCENT;
    state->elapsedMs = 0;
    state->speed = 0;
    state->course = 0;
    state->_wpIndex = 0;

    if (!config->useGPSTime) {
        state->hour = config->startHour;
        state->minute = config->startMinute;
        state->second = config->startSecond;
    }
    state->dayOfYear = config->startDayOfYear;
}

void hab_sim_update(SimState *state, const SimConfig *config, uint32_t stepMs) {
    if (state->landed) return;

    state->elapsedMs += stepMs;
    float dtSec = stepMs / 1000.0f;

    if (config->mode == SIM_MODE_WIND_LAYERS) {
        sim_update_wind_layers(state, config, dtSec);
    } else {
        sim_update_waypoints(state, config);
    }

    /* Advance simulated time if not using GPS clock */
    if (!config->useGPSTime) {
        sim_advance_time(state, stepMs);
    }
}
