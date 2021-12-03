# Dokumentation/Zusammenfassung von Kommunikation mit MCU

## SPI RFC-MCU Übersicht
- MCU Master
- RFC Slave
- In welchen Intervallen sendet die MCU am Anfang? Wie wird anfängliche Frequenz 
- Am Anfang Frequenzen für Module senden

## Datenpakete
### Standard
1. LEN (a)
2. NP (1 bit) + *Reserved* (3 bit) + ADC-Flag (1 bit) + Protokoll (3 bit) (b)
3. PS (4 bit) + ComEn (4 bit)
4. Data0 ... DataN
5. (CRC)

### Sonderpakete
1. Null
2. LEN (a)
3. Metainfo (ASCII Char.) (c)
4. String (ASCII String)
5. (CRC)
        
        
        
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
