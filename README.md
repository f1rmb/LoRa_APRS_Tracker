# LoRa APRS Tracker

The LoRa APRS Tracker will work with very cheep hardware which you can buy from amazon, ebay or aliexpress.
Try it out and be part of the APRS network.

![TTGO T-Beam](pics/Tracker.png)

## Supported boards

You can use one of the Lora32 boards:

* TTGO T-Beam V0.7 (433MHz SX1278)
* TTGO T-Beam V1 (433MHz SX1278)

This boards cost around 30 Euros, they are very cheap but perfect for an LoRa iGate.
Keep in minde: you need a 433MHz version!

## Compiling and configuration

**There is a german [quick start](https://www.lora-aprs.info/docs/LoRa_APRS_iGate/quick-start-guide/) page! Take a look ;)**

**There is a french [quick start](http://www.f5kmy.fr/spip.php?article509) page! Take a look ;)**

### How to compile

The best success is to use PlatformIO (and it is the only platform where I can support you). 

* Go to [PlatformIO](https://platformio.org/) download and install the IDE. 
* If installed open the IDE, go to the left side and klick on 'extensions' then search for 'PatformIO' and install.
* When installed click 'the ant head' on the left and choose import the project on the right.
* Just open the folder and you can compile the Firmware.

### Configuration

* You can find all nessesary settings to change for your configuration in **data/tracker.json**.
* The `button_tx` setting enables manual triggering of the beacon using the middle button on the T-Beam.
* To upload it to your board you have to do this via **Upload File System image** in PlatformIO!
* To find the 'Upload File System image' click the PlatformIO symbol (the little alien) on the left side, choos your configuration, click on 'Platform' and search for 'Upload File System image'.

### Changes in this fork

* The NMEA parsing from the GPS has been dropped, it now uses binary UBX mode (for both Neo-6M and M8N modules, personal forked Sparkfun's U-BLOX GNSS lib dependency is handled by PlatformIO)
* The average power consumption has been heavily reduced. The battery life came from ~40 hours, before the modifications, up to weeks now, depending of the settings. Thanks to the UBX mode, ESP32 light-sleeps and down clocking the board (40MHz for Neo-6M, 20MHz for M8N).
* If you don't want to mess up the **data/tracker.json**, you can create your own **data/user_tracker.json** file, which have a higher priority than **data/tracker.json**.
* In the JSON file, some extra configuration entries are available:
  - **beacons**::**...**::**add_power**: add LoRa output power (mW) to the beacon,
  - **beacons**::**...**::**lora_power** (OPTIONAL): change LoRa output power setting per beacon,
  - **location**::**\***: define your default location settings (see below, GPS ON/OFF),
  - **display**::**invert**: invert video,
  - **display**::**rotation** (0..3): screen rotation,
  - **display**::**contrast** (0.255): screen contrast (brightness),
  - **display**::**timeout**: screen saver (OFF), in seconds.
* User button actions:
  - **1** click, if the screen is OFF: lit the screen,
  - **1** click, if the screen is lit: transmit a beacon, if the GPS has a fix or if the GPS is OFF,
  - **2** clicks, toggles the screen saver ON or OFF,
  - **3** or **more** clicks: toggles the GPS modules ON or OFF (for the boards embedding an AXP chip, the GPS module is really powered off, for the other boards the module is put in power saving mode):
     * when the GPS is turned OFF while a fix was acquired, the current position will be kept. If it had no fix, the position from the JSON's location settings will be used instead.
  - **long press**: select the next beacon (see JSON beacons array):
     * if the GPS is currently OFF and you select a beacon where the lora_power setting is defined, the LoRa module will use that power setting, otherwise the **lora**::**power** value will apply.
* Uses a forked version of the esp_logger lib, which permits to drop the color escape sequences (which was annoying).
* Implement a battery voltage monitoring for v0.7 boards: if the average voltage drops below 3.27V, all devices are set to power save mode, clock is set to 10MHz then the ESP32 is going to infinite deep sleep. This avoids to kill the board (already happened)
* A star (**\***) is blinking next to the date/time when the GPS has a fix.

## LoRa iGate

Look at my other project: a [LoRa iGate](https://github.com/peterus/LoRa_APRS_iGate)
