# AxleWatch Cloud Upload Feature

## Overview

The AxleWatch-RX now supports automatic uploading of temperature and GPS data to the AxleWatch.com cloud dashboard. This allows fleet managers to monitor their vehicles remotely in real-time.

## Features

- **Automatic Data Upload**: Temperature and GPS data are automatically uploaded every 60 seconds
- **WiFi Connectivity**: Uses your configured WiFi network (Starlink or other)
- **JSON API**: Sends structured JSON data with all trailer and GPS information
- **Status Monitoring**: Web interface shows upload status and statistics
- **Error Handling**: Automatic retry logic and connection error handling

## Setup Instructions

### 1. Configure WiFi Connection

1. Connect to the AxleWatch-Setup WiFi access point from your phone or laptop
2. Navigate to `http://192.168.4.1` in your browser
3. Enter your WiFi credentials (e.g., your Starlink SSID and password)
4. Click "Save Configuration"

### 2. Configure Upload URL

1. On the same configuration page, enter your AxleWatch.com upload URL in the "Upload URL" field
   - Example: `https://axlewatch.com/api/fleet/ingest`
   - Contact AxleWatch support if you don't have this URL
2. Click "Save Configuration"

### 3. Verify Upload Status

You can check if uploads are working by:

1. Navigate to `http://192.168.4.1/api/upload/status` in your browser
2. The JSON response will show:
   ```json
   {
     "enabled": true,
     "lastUploadTime": 123456,
     "lastSuccess": true,
     "lastHttpStatus": 200,
     "uploadCount": 42,
     "secondsSinceLastUpload": 15
   }
   ```

### Status Indicators

- **enabled**: Whether cloud uploads are active
- **lastSuccess**: If the most recent upload succeeded
- **lastHttpStatus**: HTTP status code from last upload (200 = success)
- **uploadCount**: Total successful uploads since boot
- **secondsSinceLastUpload**: Time elapsed since last upload attempt

## Data Format

The system uploads the following data in JSON format:

```json
{
  "deviceId": "AABBCCDDEEFF",
  "timestamp": 123456789,
  "gps": {
    "valid": true,
    "latitude": 45.5234,
    "longitude": -122.6762,
    "speed": 65.5,
    "course": 180.0,
    "satellites": 12,
    "logging": true
  },
  "trailers": [
    {
      "id": 1,
      "online": true,
      "rssi": -65,
      "lastUpdate": 123456789,
      "ambientTemp": 25.3,
      "hubTemperatures": [45.2, 46.1, 44.8, 47.3, 45.9, 46.7, 45.4, 46.2]
    }
  ],
  "wifiRssi": -45
}
```

## Configuration

### Upload Interval

By default, data is uploaded every 60 seconds. To change this, modify the following line in `AXLEWATCH_V2_Touch__Webserver.ino`:

```cpp
cloudUpload.setUploadInterval(60);  // Change to desired seconds
```

### Device ID

The device ID is automatically generated from the WiFi MAC address. Each receiver unit will have a unique ID that identifies it on the AxleWatch.com dashboard.

## Troubleshooting

### No uploads happening

1. **Check WiFi connection**: Ensure the receiver is connected to your WiFi network
   - Access point IP shows WiFi is in AP mode only
   - Should see a station IP when connected
2. **Check upload URL**: Make sure a valid URL starting with `http://` or `https://` is configured
3. **Check serial monitor**: Look for `[CloudUpload]` messages showing upload attempts

### Uploads failing (HTTP errors)

1. **HTTP 404**: Upload URL is incorrect - verify with AxleWatch support
2. **HTTP 401/403**: Authentication required - may need API key (contact support)
3. **HTTP 500**: Server error - check AxleWatch.com status
4. **Negative error codes**: Network/connection errors - check WiFi signal strength

### View detailed logs

Connect to the ESP32 via USB and open the Serial Monitor at 115200 baud to see detailed upload logs:

```
[CloudUpload] Starting upload...
[CloudUpload] Payload size: 456 bytes
[CloudUpload] Posting to: https://axlewatch.com/api/fleet/ingest
[CloudUpload] HTTP Response code: 200
[CloudUpload] Upload successful! Total uploads: 42
```

## API Endpoints

The local web server provides these endpoints:

- **GET /** - Configuration web page
- **GET /live** - Live dashboard web page
- **GET /api/live** - Live data JSON
- **GET /api/config** - Current configuration JSON
- **GET /api/upload/status** - Cloud upload status JSON
- **GET /wifi/status** - WiFi connection status JSON
- **POST /save** - Save configuration

## Security Notes

- The upload URL is stored in ESP32 non-volatile memory
- WiFi credentials are stored securely in ESP32 Preferences
- All web traffic uses standard HTTP/HTTPS protocols
- Consider using HTTPS upload URLs for encrypted transmission

## Support

For assistance with cloud upload configuration or AxleWatch.com dashboard access, contact AxleWatch support at support@axlewatch.com or visit https://axlewatch.com/support
