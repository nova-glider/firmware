
# communications

## description of communications between server and ground-station

The communciation will happen through a POST request hat will have following __json__ data:

> __Note__: this is not exhaustive; other parameters may be added under readings.

```json
{
  "device_id": "sensor-001",
  "timestamp": "2025-06-05T14:23:45Z",
  "location": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "altitude": 15.5
  },
  "readings": {
    "temperature_celsius": 22.8,
    "humidity_percent": 58.3,
    "air_pressure_hpa": 1013.2,
    "air_quality_index": 42,
    "co2_ppm": 415
  },
  "status": {
    "battery_level_percent": 76,
    "signal_strength_dbm": -67,
    "operational": true
  }
}
```

A key will have to be provided in the header of the post request. this key can be found under the [github page](https://github.com/samvandenabeele/cansat_SBC_2026).

> __CAUTION__: this key should remain secret and thus __NOT__ hard-coded!

For the key to remain secret, C++ or other compilers can be used in github CI/CD pipelines.

This is an example post request made by python:

```python
import requests
import json

url = "https://example.com/api/sensor-data"
api_key = "YOUR_API_KEY_HERE"

data = {
    "device_id": "sensor-001",
    "timestamp": "2025-06-05T14:23:45Z",
    "location": {
        "latitude": 37.7749,
        "longitude": -122.4194,
        "altitude": 15.5
    },
    "readings": {
        "temperature_celsius": 22.8,
        "humidity_percent": 58.3,
        "air_pressure_hpa": 1013.2,
        "air_quality_index": 42,
        "co2_ppm": 415
    },
    "status": {
        "battery_level_percent": 76,
        "signal_strength_dbm": -67,
        "operational": True
    }
}

headers = {
    "Content-Type": "application/json",
    "Authorization": f"Bearer {api_key}"
}

response = requests.post(url, headers=headers, data=json.dumps(data))

print("Status Code:", response.status_code)
print("Response Body:", response.text)

```

The response should be `200, OK` (server default).

The api key will be sent from the cansat to the ground-station, and from the ground-station to the server.

If you need anymore information, please open an issue or contact @samvandenabeele
