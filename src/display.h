
#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Arduino.h>


class OLEDDisplay
{
    public:
        OLEDDisplay();
        ~OLEDDisplay();

        void Init();

        void Display(const String &header, uint32_t msPause = 0);
        void Display(const String &header, const String &line1, uint32_t msPause = 0);
        void Display(const String &header, const String &line1, const String &line2, uint32_t msPause = 0);
        void Display(const String &header, const String &line1, const String &line2, const String &line3, uint32_t msPause = 0);
        void Display(const String &header, const String &line1, const String &line2, const String &line3, const String &line4, uint32_t msPause = 0);
        void Display(const String &header, const String &line1, const String &line2, const String &line3, const String &line4, const String &line5, uint32_t msPause = 0);

        void Activate(bool activate);
        bool IsActivated();

    protected:
        void displayLines(const String &header, const String &line1 = emptyString, const String &line2 = emptyString, const String &line3 = emptyString, const String &line4 = emptyString, const String &line5 = emptyString, uint32_t msPause = 0);


    private:
        Adafruit_SSD1306   m_display;
        bool               m_isInitialized;
        bool               m_isActivated;
};
#endif
