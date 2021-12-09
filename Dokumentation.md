# Dokumentation/Zusammenfassung von Kommunikation mit MCU

## SPI RFC-MCU Übersicht
- MCU Master
- RFC Slave

## Datenpakete
### Standard
1. LEN (a)
2. NP (1 bit) + *Reserved* (3 bit) + ADC-Flag (1 bit) + Protokoll (3 bit) (b)
3. PS (4 bit) + ComEn (4 bit)
4. Data0 ... DataN (je 2 Byte)
5. (CRC)

### Sonderpakete
1. Null
2. LEN (a)
3. Metainfo (ASCII Char.) (c)
4. String (ASCII String)
5. (CRC)
        
## Payload der Module
### ODC
5 Sensoren / Data0 ... Data4 / 10 Bytes
1. Lage (normalisiert bzw. ohne Einheit)
2. Lage (Winkelgrad)
3. Fototransistor
4. IMU + Kompass
5. (Sonstiges)

### TMS
6 Temperatursensoren / Data0 ... Data5 / 12 Bytes

### EPM
4 - 5 Stromsensoren / Data0 ... Data3-4 / 8 - 10 Bytes

#
(a)
LEN = Anzahl aller Bytes, inkl. LEN und CRC, außer Null-Byte im Sonderpaket

(b)
1. 001 = Shift Register Set
2. 010 = Shift Register Get
3. 011 = Shift Register Set & Get
4. 100 = I2C
5. 101 = SPI
6. 110 = UART
7. 111 = *Reserved*

(c)
- R = Restart (kein String)
- E = Error (String optional)
- I = Intervallmessung setzen (String: Tabellenplatz | Periode (1-255 bzw. 100ms - 25,5s) | String (Standardpaket))
- S = Start (kein String)



## Offene Fragen/Themen
- In welchen Intervallen sendet die MCU am Anfang? Wie wird anfängliche Frequenz für jeweilige Module festgelegt?
- Am Anfang Frequenzen für Module senden
- Wie bestätigt RFC der MCU (und andersrum) richtige/falsche Übertragung? (extra Pin? extra Nachricht?)
- Metainfo für Module An-/Abschalten
