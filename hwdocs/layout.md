# Fizikai kialakítás (összevont, végleges állapot)

Ez a dokumentum csak a jelenlegi állapotot tartalmazza (a draft 5–12 előzményei nélkül). Ábrák (azonos x/y tájolás, 4 mm/mm skálán, a mértékegység mm):
- `lid_layout.svg`, `lid_wiring.svg`: fedél, elölnézet (elrendezés / vezetékezés)
- `base_layout.svg`, `base_wiring.svg`: doboz alja, felülnézet fedél nélkül
- `box_layout_A4.pdf`: ugyanezek A4-en, 1:1 méretarányban (4 oldal)

Az elektromos kapcsolatot a `pinout_pwr.txt` (teljesítmény), a `pinout_it.txt` (jelek, I2C, UART, DHT, PZEM) adja; a csatornánkénti shunt/fuse: `shunt_fuse.txt`, `SHUNT_CALC.md`.

## Méretek
Használható belső méret: 190 × 145 mm, 4 sarokban 13 × 13 mm csavarhely. Fedél ~20 mm, doboztest 50 mm mély.

| Elem | Méret (mm) |
|---|---|
| INA3221 modul | 38 × 32, csatlakozók a hosszú oldalon, 3 × ~10 × 5 blokk, onboard shunt |
| Busbar | 88 × 13, 34 mag. (12 furat) |
| Sorkapocs | 112 × 20 × 16, 14 poz.; névleges áram 24 A |
| Biztosítéktartó | 27 × 9 × 17, forrasztható fül |
| DC/DC, 5 V kimenet: ESP32-hez 17 × 25, RPi-hez 17 × 30; < 6 mag.; bemenet a felső, kimenet és GND az alsó élen (feltételezés, forgatható) |
| ESP32-C6 Supermini | 27 × 19 × 10 |
| Kapcsoló LVSR-12V | Ø20, ~25 mély, 16 A |

## Fedél (elölnézet)
- Piros busbar (12 V) 88 × 13, x 51–139, y 4,5–17,5; a fedélsík alá 14 mm-re benyúlik.
- 15 biztosítéktartó, 9,5 mm osztás, y 31,5–58,5; felső fül → piros busbar, alsó fül → a kapcsoló bemenete. Kötési zónák: felül 8 mm (23,5–31,5, a busbar 6 mm-es zónájával határos), alul 8 mm (58,5–66,5).
- Biztosítéksor jobb szélén, 6 mm elválasztással: a LED 0 V biztosítéka (külön színnel), **1 A** (a 15 kapcsoló világítása ~50 mA/db, összesen ~750 mA; a korábbi 0,5 A nem elég).
- 15 kapcsoló 5 × 3 rácsban: sorközéppont y = 80,5 / 104,5 / 128,5 (24 mm osztás), oszlopközéppont x = 27,5 + 33,75·j; idx = 3·oszlop + sor; a körök között ≥ 4 mm; benyúlás 5 mm.
- Kötegsáv x 105–118,75: 15 SW12V (0,75–1,5 mm², a 12–14. vonal 2,5 mm²) + 1 db 0 V a LED-ekhez.
- A 12 V sín csak a 12–14. vonalon (akku/töltő/PSU) kap betáplálást.

## Doboz alja
Felül 8 mm tiltott sáv (ESP32, DC/DC-k, INA5 y ≥ 8). Az ESP32 és a DC/DC-k körül 4 mm szabad zóna.
- ESP32: x 34–61, y 8–27. DC/DC ESP32: x 65–82, y 8–33 (17 × 25). DC/DC RPi: x 86–103, y 8–38.
- INA5: x 122–160, y 8–40, vízszintes.
- INA1–4: egy sorban, y 49,5–81,5, x 3–161, köztük 2 mm; adat a felső, mérőcsatlakozó az alsó élen.
- Sorkapocs: 14 poz., x 36–148, y 103,25–123,25; 4 mm zóna a két hosszanti oldalán. A 12–14. vonalnál az áram 10–13 A (névleges 24 A rendben).
- 0 V busbar (kék, OUT_MINUS): jobbra középre, függőlegesen, x 172,5–185,5, y 28,5–116,5; a csatlakozók balra néznek; 6 mm csatlakozási zóna; 22 × 35 mm rögzítőlemez a közepén (a fedél piros busbarján is).
- Köteglefutás: x 103–123, y 83–99.
- Külön teljesítmény-csatlakozó nincs.

## Vezetékezés (koncepció)
Színek: kék = adat/jel (I2C_0, I2C_1, DHT), fekete = 0 V, narancs = terhelés (0–11. vonal, DC/DC), vörös vastag = nagy áram (12 V sín, 12–14. vonal). A rajz a sorrendet és a folyosókat adja, a keresztmetszet nem arányos.
- A 0 V törzsvezeték a bal oldali folyosón jut az ESP-hez és a DC/DC-khez; a külső visszatérők közvetlenül a kék busbarra.
- Az ESP32 tápja: DC/DC ESP32 (5,2 V) → soros 1N4001 (anód a DC/DC felől, katód/csík a mérő felé) → INA1 ch1 shunt → ESP32 „5V" láb. A Supermini „B+/B−" akkulábait nem használjuk, 5 V-ot oda tilos adni.
- Az RPi külön DC/DC-ről kap tápot (INA1 ch2).
- A 12–14. vonal SW12V-ja felfelé az INA5-höz és vissza a sorkapocsra a jobb oldali folyosón (x 170–190) fut, 6 db ≥ 2,5 mm².

## Külső csatlakozók
- DHT1/2: 2 × 3 pólusú csatlakozó az ESP mellett (3V3, GND, adat) + tömszelence; a DHT-k nem a dobozban vannak (az önmelegedés miatt).
- PZEM-004T (AC mérő): külső modul, 4 pólusú csatlakozó (3V3, GND, UART_1_TX = GPIO2, UART_1_RX = GPIO3) + tömszelence. A modul hálózati oldala nem kerül a dobozba, csak az optocsatolós TTL oldal.
  - BAT42 modulonként a modul-oldali TX érintkezőn: katód (csík) a PZEM TX felé, anód a közös RX vonal felé.
  - 10 kΩ felhúzó a UART_1_RX vonalon a 3V3-ra (egyszer a vonalra), 330 Ω soros ellenállás az ESP GPIO3 előtt.
  - Több PZEM modul ebben a dobozban nincs tervezve (lásd `config.h` megjegyzés).

## Külső rögzítés
- Rögzítő téglalap 100 × 147 mm; a 147 a 200 mm-es oldal mentén.
- Furatközéppontok a belső koordinátarendszerben: (21,5; 22,5), (168,5; 22,5), (21,5; 122,5), (168,5; 122,5); 4 db, erősített; mindegyik körül Ø16 mm tiltott zóna (a vezeték átfuthat felette).
- A kék busbar két vége a jobb oldali zónákkal ütközik: elfogadott (a rögzítőlemez a doboz rögzítésekor kivehető).

## Magasság
Doboztest elemei a fedélsík alatt: kék busbar 34 mm, sorkapocs 16, ESP 10, INA ~10 (távtartóval). A fedél alatt kilógók: piros busbar 14 mm, kapcsolók 5 mm; ütközés nincs.

## Állapot, nyitott pontok
- Végleges: BTR 0 és CHARGER shunt (2 × 0,02 Ω, T-12,5A); sorkapocs 24 A.
- Ideiglenes: BTR 1 shunt (1 × 0,02 Ω, T-5A).
- Mérendő: az RPi tényleges terhelése (az INA1 ch2, 25 mΩ/T-2,5A felülvizsgálata), az RPi DC/DC hőterhelése próbaüzem után.
- Ellenőrizendő: a Supermini „5V" lábának és az USB tápjának összekötése (ha az USB és az „5V" láb dióda nélkül közös, USB-csatlakoztatáskor a soros 1N4001 a DC/DC felé védelmet ad; a Supermini kapcsolási rajza alapján megerősítendő).
- A DC/DC-k lábkiosztása és tájolása feltételezett, a tényleges modul alapján pontosítandó.
