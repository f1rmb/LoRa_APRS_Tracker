
#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Arduino.h>

#define USE_BOOTSCREEN

class Timer
{
    public:
        Timer(uint32_t timeoutInMs) :
            m_running(false),
            m_paused(false),
            m_timeout(timeoutInMs),
            m_timer(0),
            m_lastMillis(0U)
        {
        }

        Timer() : Timer(0U)
        {
        }

        ~Timer()
        {
        }

        void setTimeout(uint32_t timeoutMS)
        {
            m_timeout = timeoutMS;
        }

        void start()
        {
            if (m_timeout > 0U)
            {
                m_timer = m_timeout;
                m_lastMillis = millis();
                m_running = true;
                m_paused = false;
            }
        }

        void stop()
        {
            m_timer = 0;
            m_running = false;
            m_paused = false;
        }

        void pause()
        {
            if (m_running && (m_paused == false))
            {
                m_paused = true;
            }
        }

        void resume()
        {
            if (m_running && m_paused)
            {
                m_paused = false;
            }
        }

        bool isRunning()
        {
            return m_running;
        }

        bool isPaused()
        {
            return (m_running && m_paused);
        }

        bool hasExpired()
        {
            if (m_running && (m_timer <= 0))
            {
                return true;
            }

            return false;
        }

        void clock()
        {
            if (m_running && (m_timer > 0))
            {
                unsigned long ms = millis();
                unsigned long span = (ms - m_lastMillis);

                if (span > 0U)
                {
                    if (m_paused == false)
                    {
                        m_timer = std::max((m_timer - int32_t(span)), 0);
                    }

                    m_lastMillis = ms;
                }
            }
        }

    private:
        bool            m_running;
        bool            m_paused;
        uint32_t        m_timeout;
        int32_t         m_timer;
        unsigned long   m_lastMillis;
};

class OLEDDisplay
{
    public:
        typedef enum
         {
             SCREEN_MODE_APRS,
             SCREEN_MODE_SENSORS,
             SCREEN_MODE_MAX
         } ScreenMode_t;

    public:
        OLEDDisplay();
        ~OLEDDisplay();

        void Init(bool invert, uint8_t rotation, uint8_t contrast, ScreenMode_t screenModeStartup, uint32_t screenModeDurationInSec);
        void setContrast(uint8_t contrast);

#if defined(USE_BOOTSCREEN)
        void ShowBootscreen(const String &version, uint16_t x, uint16_t y, uint16_t textcolor, uint32_t msPause = 0);
#endif

        void Display(const String &header, uint32_t msPause = 0);
        void Display(const String &header, const String &line1, uint32_t msPause = 0);
        void Display(const String &header, const String &line1, const String &line2, uint32_t msPause = 0);
        void Display(const String &header, const String &line1, const String &line2, const String &line3, uint32_t msPause = 0);
        void Display(const String &header, const String &line1, const String &line2, const String &line3, const String &line4, uint32_t msPause = 0);
        void Display(const String &header, const String &line1, const String &line2, const String &line3, const String &line4, const String &line5, uint32_t msPause = 0);

        void Activate(bool activate);
        bool IsActivated();

        ScreenMode_t GetScreenMode();

        void Tick();

    protected:
        void displayLines(const String &header, const String &line1 = emptyString, const String &line2 = emptyString, const String &line3 = emptyString, const String &line4 = emptyString, const String &line5 = emptyString, uint32_t msPause = 0);


    private:
        Adafruit_SSD1306   m_display;
        bool               m_isInitialized;
        bool               m_isActivated;
        uint8_t            m_contrast;
        ScreenMode_t       m_screenMode;
        Timer              m_screenModeTimer;
        bool               m_firstRun;
};
#endif
