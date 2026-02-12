#include "GPS.h"

using namespace astra;

#pragma region GPS Specific Functions

GPS::GPS(const char *name) : Sensor(name)
{
    hr = 0;
    min = 0;
    sec = 0;
    snprintf(tod, 9, "%s", "00:00:00");
    addColumn("%0.7f", &position.x(), "Lat (deg)");
    addColumn("%0.7f", &position.y(), "Lon (deg)");
    addColumn("%0.3f", &position.z(), "Alt (m)");
    addColumn("%0.3f", &velocity.x(), "vN (m/s)");
    addColumn("%0.3f", &velocity.y(), "vE (m/s)");
    addColumn("%0.3f", &velocity.z(), "vD (m/s)");
    addColumn("%d", &fixQual, "Fix Quality");
    addColumn("%s", &tod, "Time of Day");
    hasFirstFix = false;
    hasFix = false;
    setUpdateRate(5);
}

GPS::~GPS() {}

Vector<3> GPS::getPos() const { return position; }

Vector<3> GPS::getVel() const { return velocity; }

double GPS::getHeading() const { return heading; }

bool GPS::getHasFix() const { return hasFix; }

int GPS::getFixQual() const { return fixQual; }

// Taken from this article and repo. As I understand it, it's an accurate approximation of the Vincenty formulae to find the distance between two points on the earth
//  https://github.com/mapbox/cheap-ruler/blob/main/index.js#L475
//  https://blog.mapbox.com/fast-geodesic-approximations-with-cheap-ruler-106f229ad016
void GPS::calcInitialValuesForDistance()
{
    constexpr auto EARTH_RAD = 6378.137e3;               // meters
    constexpr auto RAD = 3.14159265358979323846 / 180.0; // lol

    constexpr auto EARTH_FLAT = 1.0 / 298.257223563; // flattening of the earth. IDK what this means

    constexpr auto ECC_SQRD = EARTH_FLAT * (2.0 - EARTH_FLAT); // eccentricity squared. IDK what this means

    constexpr auto m = RAD * EARTH_RAD;
    const auto coslat = cos(position.x() * RAD);
    const auto w2 = 1.0 / (1.0 - ECC_SQRD * (1.0 - coslat * coslat)); // IDK what this means
    const auto w = sqrt(w2);                                          // IDK what this means

    ky = m * w * coslat;                // IDK what this means
    kx = m * w * w2 * (1.0 - ECC_SQRD); // IDK what this means
}

Vector<3> GPS::getDisplacement(Vector<3> origin) const
{
    Vector<3> displacement;
    double dy = wrapLongitude(position.y() - origin.y()) * ky;
    double dx = (position.x() - origin.x()) * kx;
    displacement.x() = dx;
    displacement.y() = dy;
    displacement.z() = (position.z() - origin.z());
    return displacement;
}

double GPS::wrapLongitude(double val) const
{
    while (val > 180)
        val -= 360;
    while (val < -180)
        val += 360;
    return val;
}

#pragma region Time Functions

int8_t GPS::getHour() const { return hr; }
int8_t GPS::getMinute() const { return min; }
int8_t GPS::getSecond() const { return sec; }

uint8_t GPS::getDay() const { return day; }
uint8_t GPS::getMonth() const { return month; }
uint16_t GPS::getYear() const { return year; }

const char *GPS::getTimeOfDay() const { return tod; }

void GPS::findTimeZone()
{
    bool isDST = (month > 3 && month < 11) || (month == 3 && day >= 10) || (month == 11 && day < 3);
    if (isDST)
    {
        hrOffset = 1;
        LOGI("DST is in effect.");
    }
    else
    {
        hrOffset = 0;
        LOGI("DST is not in effect.");
    }

    if (getPos().x() > -82.5)
    {
        hrOffset -= 5;
        LOGI("Timezone: Eastern Standard Time");
    }
    else if (getPos().x() > -97.5)
    {
        hrOffset -= 6;
        LOGI("Timezone: Central Standard Time");
    }
    else if (getPos().x() > -112.5)
    {
        hrOffset -= 7;
        LOGI("Timezone: Mountain Standard Time");
    }
    else if (getPos().x() > -127.5)
    {
        hrOffset -= 8;
        LOGI("Timezone: Pacific Standard Time");
    }
    else if (getPos().x() > -135)
    {
        hrOffset -= 9;
        LOGI("Timezone: Alaska Standard Time");
    }
    else if (getPos().x() > -150)
    {
        hrOffset -= 10;
        LOGI("Timezone: Hawaii-Aleutian Standard Time");
    }
    else
    {
        LOGI("Timezone: UTC");
    }
    LOGI("Timezone offset: %d", hrOffset);
}

#pragma endregion // GPS Specific Functions

#pragma region Sensor Virtual Function Implementations

int GPS::update(double currentTime)
{
    if (!initialized)
        return -1;

    int err = read();

    if (err != 0)
    {
        healthy = false;
        return err;
    }

    if (!hasFix && fixQual >= 4)
    {
        hasFix = true;
        // getEventManager().invoke(GPSFix{"GPS_FIX"_i, this, !hasFirstFix}); // will be false the first time this runs, so invert it
        if (!hasFirstFix)
        {
            findTimeZone();
            hasFirstFix = true;
        }
        calcInitialValuesForDistance();
    }
    if (hasFix)
    {
        if (fixQual < 4)
        {
            hasFix = false;
            // getEventManager().invoke(GPSFix{"GPS_FIX"_i, this, false});
        }
        hr += hrOffset;
        hr = (hr % 24 + 24) % 24; // in cpp -1 % 24 = -1, but we want it to be 23
        min = min % 60;
        sec = sec % 60;                                    // just in case
        snprintf(tod, 12, "%02d:%02d:%02d", hr, min, sec); // size is really 9 but 12 ignores warnings about truncation. IRL it will never truncate
    }

    updateHealth();
    return 0;
}

void GPS::updateHealth()
{
    // GPS health is simply based on successful read, not fix status.
    // No fix is expected behavior (not a hardware failure), so we keep healthy=true.
    // Health only becomes false on read() errors (handled in update() before this is called).
    healthy = true;
}

int GPS::begin()
{
    position = Vector<3>(0, 0, 0);
    velocity = Vector<3>(0, 0, 0);
    fixQual = 0;
    hasFix = false;
    hasFirstFix = false;
    heading = 0;
    int err = init();
    initialized = (err == 0);
    return err;
}

#pragma endregion // Sensor Virtual Function Implementations