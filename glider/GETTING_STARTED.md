# Getting started programming for cansat, a concise look into STM32 for CanSat

## BME280

```c
BME280Calculation(&BME280);
```

```c
BME280.Temperature;
BME280.Pressure;
BME280.Humidity;
BME280.AltitudeTP;
BME280.AltitudeP;
```

# RFM98

```c
RF98_send();
```

## MPU9250

```c
MPU_readProcessedData(&hi2c1);
```

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

```c
MPU_calcAttitude();
```

```c
attitude
```

## GPS

```c
nmea_parse()
```
