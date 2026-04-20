# Getting started programming for cansat, a concise look into STM32 for CanSat

## BME280

om data uit de BME te halen, doe je altijd eerst

```c
BME280Calculation(&BME280);
```

waarna je aan de data kan door deze variabelen aan te spreken:

```c
BME280.Temperature;
BME280.Pressure;
BME280.Humidity;
BME280.AltitudeTP;
BME280.AltitudeP;
```

# RFM98

Om een boodschap te sturen met de radiomodule, kan je deze gebruiken:

```c
RF98_send();
```

Je zal de string wel naar het juiste datatype moeten casten, maar da's niet zo moeilijk

## MPU9250

bij de MPU hetzelfde als de BME280, eerst:

```c
MPU_readProcessedData(&hi2c1);
```

dan kan je de data aanspreken met:

```c
MPURawdata.ax;
MPURawdata.ay;
MPURawdata.az;

MPURawdata.gx;
MPURawdata.gy;
MPURawdata.gz;
```

```c
MPUdata.ax;
MPUdata.ay;
MPUdata.az;

MPUdata.gx;
MPUdata.gy;
MPUdata.gz;
```

je kan uit de MPU ook wat extra data halen door wat leuke berekeningetjes die de library doet voor ons. Eerst pop je deze:

```c
MPU_calcAttitude();
```

hierna krijg je de effectieve orientatie van de cansat tegenover de aarde, in radialen (vermoed ik). Deze kan je dan zo aanspreken:

```c
MPUattitude.p // voor de pitch
MPUattitude.r // voor de roll
MPUattitude.y // voor de yaw
```

Hoe je deze data exact gebruikt moet je maar eens uitvogelen, en in welke orientatie dat ze nul geeft etc (weet ook: het kan zijn dat de assen niet overeen komen met de werkelijkheid. Dit komt door de orientatie van de chip op de pcb)

## GPS

GPS is super cool en handig! Eerst gebruik je dit functietje:

```c
nmea_parse(&GPSData, GPSBuffer);
```

daarna kan je aan de gpsdata:

```c
GPSData.latitude // latitude in degrees with decimal places
GPSData.latSide // N or S
GPSData.longitude // longitude in degrees with decimal places
GPSData.lonSide // E or W
GPSData.altitude // altitude in meters
GPSData.hdop // horizontal dilution of precision
GPSData.satelliteCount // number of satellites used in measurement
GPSData.fix // 1 = fix, 0 = no fix
GPSData.lastMeasure // hhmmss.ss UTC of last successful measurement; time
```

en als laatste: Veel van deze functies verwachten de peripheral die ze moeten gebruiken. Welke bij welke hoort weet ik niet meer, maar dat kan je opzoeken in de schematjes die je normaal gezien al van mij gekregen hebt (indien niet, bel of stuur een berichtje)

Het enige dat nog toegevoegd hoort te worden wijn de libraries voor de micro-sd kaart.

Het is voor dit soort werk ook wel handig om in branches te werken als je zelf nog drivers wilt toevoegen.

En tot slot: veel geluk!
