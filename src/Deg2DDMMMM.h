#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

typedef struct
{
        bool      negative;
        double    value;
        uint16_t  degrees;
        double    minutes; // full precision, no decimation
        uint16_t  hundredths; // * 1e-2
        uint16_t  millionths; // millionth (2 digits only)
} Deg2DDMMMMPosition;


class Deg2DDMMMM
{
    public:
        Deg2DDMMMM() {}
        ~Deg2DDMMMM() {}

        static void Convert(Deg2DDMMMMPosition &p, double d, bool highPrecision)
        {
            double decimation = 0.0;

            p.negative = std::signbit(d);                          // Negative value ?
            p.value = fabs(d);                                     // Absolute value
            p.degrees = uint16_t(p.value);                         // integer part
            p.minutes = (p.value - p.degrees) * 60.0;              // Fractionnal part, converted to minutes

            if (highPrecision)
            {
                decimation = (p.minutes * 1e2); // avoid rounding
                p.millionths = uint16_t(nearbyint((decimation - int(decimation)) * 1e2)); // Extract pure value for DAO: mm.mm[mm]
            }
            else
            {
                decimation = nearbyint(p.minutes * 1e2); // rounding
                p.millionths = 0; // DAO = 00
            }

            p.hundredths = uint16_t(decimation);                   // minutes * 100, for integer storage.

#if 0
            printf("p.minutes: %f, p.hundredths: %u (%f)\n", p.minutes, p.hundredths, p.hundredths * 1e-2);

            printf("Value: %f , NMEA(APRS): %0.*u%05.2f[%02u] (minutes: %f)\n", p.value, (isLongitude ? 3 : 2), p.degrees, (p.hundredths * 1e-2), p.millionths, p.minutes);

            // Checking
            uint16_t scaledDAO = uint16_t(double(p.millionths) / 1.10) + 33;
            printf("DAO %u => %u '%c'\n\n", p.millionths, scaledDAO, scaledDAO);
            double offset = ((double(scaledDAO) - 33.0) / 91.0 * 0.01);
            double checkPos = (p.degrees + (((p.hundredths * 1e-2) + offset) / 60.0)) * (p.negative ? -1.0 : 1.0);
            printf("chk: %f (%f) (offset: %f)\n", checkPos, p.value, offset);
            printf("====\n");
#endif
        }

        static char *Format(char *dest, const Deg2DDMMMMPosition &p, bool isLongitude)
        {
            sprintf(dest, "%.*u%05.2f%c", (isLongitude ? 3 : 2), p.degrees, (p.hundredths * 1e-2), (isLongitude ? (p.negative ? 'W' : 'E') : (p.negative ? 'S' : 'N')));
            return dest;
        }

        static char *DAO(char *dest, const Deg2DDMMMMPosition &pLat, const Deg2DDMMMMPosition &pLong)
        {
            sprintf(dest, "!w%c%c!", uint16_t(double(pLat.millionths) / 1.10) + 33, uint16_t(double(pLong.millionths) / 1.10) + 33);
            return dest;
        }

};
