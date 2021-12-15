
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <logger.h>

#include "display.h"
#include "pins.h"

static String currentStrings[6]; // header + 5 lines

OLEDDisplay::OLEDDisplay() :
m_display(128, 64, &Wire, OLED_RST),
m_isInitialized(false),
m_isActivated(false)
{
    for (size_t i = 0; i < (sizeof(currentStrings) / sizeof(currentStrings[0])); i++)
    {
        currentStrings[i] = emptyString;
    }
}

OLEDDisplay::~OLEDDisplay()
{
}

// cppcheck-suppress unusedFunction
void OLEDDisplay::Init()
{
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(20);
    digitalWrite(OLED_RST, HIGH);

    Wire.begin(OLED_SDA, OLED_SCL);
    if (!m_display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false))
    {
        logPrintlnE("SSD1306 allocation failed");
        while (true) { }
    }

    m_display.clearDisplay();
    m_display.setTextColor(WHITE);
    m_display.setTextSize(1);
    m_display.setCursor(0, 0);
    m_display.print("LORA SENDER ");
    m_display.ssd1306_command(SSD1306_SETCONTRAST);
    m_display.ssd1306_command(1);
    m_display.display();
    m_isInitialized = true;
    m_isActivated = true;
}

void OLEDDisplay::displayLines(const String &header, const String &line1, const String &line2, const String &line3, const String &line4, const String &line5, uint32_t msPause)
{
    if (m_isInitialized == false)
    {
        return;
    }

    m_display.clearDisplay();
    m_display.setTextColor(WHITE);
    m_display.setTextSize(2);
    m_display.setCursor(0, 0);
    m_display.println(header);

    if (line1 != emptyString)
    {
        m_display.setTextSize(1);
        m_display.setCursor(0, 16);
        m_display.println(line1);
    }

    if (line2 != emptyString)
    {
        m_display.setCursor(0, 26);
        m_display.println(line2);
    }

    if (line3 != emptyString)
    {
        m_display.setCursor(0, 36);
        m_display.println(line3);
    }

    if (line4 != emptyString)
    {
        m_display.setCursor(0, 46);
        m_display.println(line4);
    }

    if (line5 != emptyString)
    {
        m_display.setCursor(0, 56);
        m_display.println(line5);
    }

    m_display.ssd1306_command(SSD1306_SETCONTRAST);
    m_display.ssd1306_command(1);
    m_display.display();

    // Store currently displayed strings.
    currentStrings[0] = header;
    currentStrings[1] = line1;
    currentStrings[2] = line2;
    currentStrings[3] = line3;
    currentStrings[4] = line4;
    currentStrings[5] = line5;

    delay(msPause);
}

// cppcheck-suppress unusedFunction
void OLEDDisplay::Display(const String &header, uint32_t msPause)
{
    displayLines(header, emptyString, emptyString, emptyString, emptyString, emptyString, msPause);
}

// cppcheck-suppress unusedFunction
void OLEDDisplay::Display(const String &header, const String &line1, uint32_t msPause)
{
    displayLines(header, line1, emptyString, emptyString, emptyString, emptyString, msPause);
}

// cppcheck-suppress unusedFunction
void OLEDDisplay::Display(const String &header, const String &line1, const String &line2, uint32_t msPause)
{
    displayLines(header, line1, line2, emptyString, emptyString, emptyString, msPause);
}

// cppcheck-suppress unusedFunction
void OLEDDisplay::Display(const String &header, const String &line1, const String &line2, const String &line3, uint32_t msPause)
{
    displayLines(header, line1, line2, line3, emptyString, emptyString, msPause);
}

// cppcheck-suppress unusedFunction
void OLEDDisplay::Display(const String &header, const String &line1, const String &line2, const String &line3, const String &line4, uint32_t msPause)
{
    displayLines(header, line1, line2, line3, line4, emptyString, msPause);
}

// cppcheck-suppress unusedFunction
void OLEDDisplay::Display(const String &header, const String &line1, const String &line2, const String &line3, const String &line4, const String &line5, uint32_t msPause)
{
    displayLines(header, line1, line2, line3, line4, line5, msPause);
}

void OLEDDisplay::Activate(bool activate)
{
    if (m_isInitialized == false)
    {
        return;
    }

    if (activate != m_isActivated)
    {
        m_display.ssd1306_command(activate ? SSD1306_DISPLAYON : SSD1306_DISPLAYOFF);
        m_display.ssd1306_command(1);

        if (activate) // Restore the screen
        {
            displayLines(currentStrings[0], currentStrings[1], currentStrings[2], currentStrings[3], currentStrings[4], currentStrings[5]);
        }

        m_isActivated = activate;
    }
}

bool OLEDDisplay::IsActivated()
{
    return (m_isInitialized && m_isActivated);
}

