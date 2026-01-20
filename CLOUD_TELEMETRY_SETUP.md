# AxleWatch Cloud Telemetry System

## Overview

The AxleWatch cloud telemetry system allows multiple users to monitor their fleet receivers from anywhere using unique MAC address identifiers. Receivers upload temperature and GPS data to the cloud, and users can view this data on the dashboard using the receiver's MAC address.

## How It Works

1. **Receivers upload data** → Cloud API stores it in memory
2. **Dashboard fetches data** → Using receiver MAC address as identifier
3. **Multiple users** → Each user adds their own receivers by MAC address

## API Endpoints

### POST /api/telemetry

**Purpose:** Receivers POST telemetry data to this endpoint

**Authentication:** Bearer token (API key)

**Headers:**
```
Authorization: Bearer <API_KEY>
Content-Type: application/json
```

**Request Body:**
```json
{
  "device_id": "AW-7C9EBD0A1F23",
  "timestamp": "2025-01-01T12:34:56Z",
  "trailer_id": "TRAILER1",
  "readings": {
    "hub_1": 45.2,
    "hub_2": 46.1,
    "hub_3": 44.8,
    "hub_4": 45.5,
    "hub_5": 46.3,
    "hub_6": 45.9,
    "hub_7": 0.0,
    "hub_8": 0.0,
    "ambient_temp": 22.5
  },
  "location": {
    "latitude": -32.123456,
    "longitude": 115.987654,
    "speed": 88.0
  },
  "alert": {
    "level": "warning",
    "message": "High temperature detected: 65.5°C"
  }
}
```

**Note:** The `alert` field is optional and only included when temperatures exceed warning/critical thresholds.

**Response (Success):**
```json
{
  "success": true,
  "deviceId": "AW-7C9EBD0A1F23",
  "timestamp": "2025-01-01T12:34:56Z"
}
```

**Response (Error - Missing device_id):**
```json
{
  "error": "Missing device_id. Please update receiver firmware to include device_id in payload.",
  "hint": "Add doc['device_id'] = configMgr->config.deviceID; to the firmware"
}
```

### GET /api/telemetry/[deviceId]

**Purpose:** Dashboard fetches telemetry data for a specific receiver

**Parameters:**
- `deviceId` - The receiver's MAC address (e.g., AW-7C9EBD0A1F23)

**Authentication:** None (public endpoint)

**Response (Success):**
```json
{
  "deviceId": "AW-7C9EBD0A1F23",
  "lastUpdate": 1704110096000,
  "lastUpdateISO": "2025-01-01T12:34:56.000Z",
  "isStale": false,
  "readingsCount": 42,
  "latestReading": {
    "timestamp": "2025-01-01T12:34:56Z",
    "trailer_id": "TRAILER1",
    "readings": { ... },
    "location": { ... },
    "alert": { ... }
  },
  "readings": [ ... ] // Array of up to 100 most recent readings
}
```

**Response (Error - Not Found):**
```json
{
  "error": "No telemetry data found for this device",
  "deviceId": "AW-7C9EBD0A1F23",
  "hint": "Make sure the receiver is online and sending data to the cloud"
}
```

## Required Firmware Changes

### 1. Add device_id to Telemetry Payload

The current firmware doesn't include the `device_id` in the cloud upload. You need to add this field so the backend knows which receiver sent the data.

### 2. Add ambient_temp to Readings

The raw LoRa data from trailers includes ambient temperature as the 10th value:
```
TX001:45.2,46.1,0.0,0.0,44.8,45.5,0.0,0.0,0.0,22.5
       ^hub1 ^hub2 ^hub3 ^hub4 ^hub5 ^hub6 ^hub7 ^hub8 ^unused ^ambient
```

When building the JSON payload, extract and include `ambient_temp` in the readings object:

```cpp
// Parse the 10th value as ambient temperature
float ambientTemp = values[9];  // 10th value (index 9)

// Add to readings object
JsonObject readings = doc.createNestedObject("readings");
readings["hub_1"] = values[0];
readings["hub_2"] = values[1];
// ... hub_3 through hub_8 ...
readings["ambient_temp"] = ambientTemp;  // Add ambient temperature
```

**Location:** `AXLEWATCH_V2_Touch__Webserver.ino` (in the cloud upload function)

**Add this line when building the JSON payload:**

```cpp
// Add device ID to the payload
doc["device_id"] = configMgr->config.deviceID;
```

**Example context:**
```cpp
void sendCloudData() {
  StaticJsonDocument<1024> doc;

  // Add device ID (NEW!)
  doc["device_id"] = configMgr->config.deviceID;

  // Existing fields
  doc["timestamp"] = getISOTimestamp();
  doc["trailer_id"] = "TRAILER1";

  // ... rest of the payload
}
```

### 3. Update Default Cloud Endpoint

Update the default cloud endpoint to point to your production domain:

```cpp
// Before:
strcpy(config.cloudEndpoint, "https://axlewatch.com/api/telemetry");

// After (if using Vercel or custom domain):
strcpy(config.cloudEndpoint, "https://your-domain.com/api/telemetry");
```

### 4. Configure API Key on Receiver

Users need to configure an API key on each receiver:

1. Connect to the receiver's config portal at `http://192.168.4.1`
2. Enter an API key in the "API Key" field (can be any non-empty string for now)
3. Click "Save Configuration"

**Note:** In production, you should validate API keys against a database of authorized keys. Currently, the system accepts any non-empty API key.

## Dashboard Usage

### For End Users

1. **Find your receiver's MAC address:**
   - Open `http://192.168.4.1` in your browser
   - Look for "Device ID" (e.g., `AW-7C9EBD0A1F23`)

2. **Add receiver to dashboard:**
   - Go to `https://axlewatch.com/dashboard`
   - Enter the Device ID or just the MAC part (e.g., `7C9EBD0A1F23`)
   - Click "Add Receiver"

3. **View telemetry data:**
   - Dashboard automatically fetches data every 10 seconds
   - See GPS location, speed, trailer ID, and hub temperatures
   - Alerts appear when temperatures exceed thresholds

### MAC Address Formats Accepted

The dashboard accepts these formats:
- `AW-7C9EBD0A1F23` (full Device ID with prefix)
- `7C9EBD0A1F23` (just the MAC address, prefix added automatically)
- `7c9ebd0a1f23` (lowercase, automatically converted to uppercase)
- `7C:9E:BD:0A:1F:23` (with colons, automatically cleaned)
- `7C-9E-BD-0A-1F-23` (with dashes, automatically cleaned)

All formats are normalized to `AW-XXXXXXXXXXXX` internally.

## Data Storage

**Current Implementation:**
- In-memory storage (data lost on server restart)
- Stores last 100 readings per device
- No persistence between deployments

**Production Recommendations:**
- Replace `lib/telemetryStore.ts` with a database implementation
- Use PostgreSQL, MongoDB, or similar
- Add proper authentication/authorization
- Implement API key management system
- Add user accounts and device ownership

## Temperature Alert Thresholds

**Dashboard Color Coding:**
- 🔴 Red (Critical): > 80°C
- 🟠 Red background: > 60°C
- 🟡 Yellow: > 50°C
- 🟢 Green: < 50°C

**Firmware should send alerts when:**
- Warning: Temp > 60°C (or your configured threshold)
- Critical: Temp > 80°C (or your configured threshold)

## Testing

### Test Receiver Upload

Use curl to simulate a receiver upload:

```bash
curl -X POST https://axlewatch.com/api/telemetry \
  -H "Authorization: Bearer test-api-key-123" \
  -H "Content-Type: application/json" \
  -d '{
    "device_id": "AW-TEST123456",
    "timestamp": "2025-01-01T12:34:56Z",
    "trailer_id": "TEST-TRAILER",
    "readings": {
      "hub_1": 45.2,
      "hub_2": 46.1,
      "hub_3": 44.8,
      "hub_4": 45.5,
      "hub_5": 46.3,
      "hub_6": 45.9,
      "ambient_temp": 22.5
    },
    "location": {
      "latitude": -32.123456,
      "longitude": 115.987654,
      "speed": 88.0
    }
  }'
```

### Test Dashboard Fetch

Open the dashboard and add `AW-TEST123456` to see the test data.

Or use curl:

```bash
curl https://axlewatch.com/api/telemetry/AW-TEST123456
```

## Security Notes

**Current Implementation (Development):**
- ⚠️ Accepts any non-empty API key
- ⚠️ No user authentication on dashboard
- ⚠️ All telemetry data is publicly accessible via device ID

**Production Recommendations:**
1. **API Key Management:**
   - Store valid API keys in database
   - Associate each key with a user account
   - Validate keys against database on POST

2. **User Authentication:**
   - Require login to view dashboard
   - Users can only see their own receivers
   - Implement device ownership/authorization

3. **Rate Limiting:**
   - Limit POST requests per API key
   - Prevent abuse of public GET endpoints

4. **Data Privacy:**
   - Add authentication to GET endpoints
   - Encrypt sensitive data
   - Add audit logging

## Deployment

The cloud API is automatically deployed with the Next.js application. No additional infrastructure required for the basic in-memory version.

For production with database:
1. Set up PostgreSQL or MongoDB
2. Update `lib/telemetryStore.ts` to use database
3. Add environment variables for DB connection
4. Deploy to Vercel/Netlify/AWS/etc.

## Support

For issues or questions:
- Email: support@axlewatch.com
- GitHub Issues: https://github.com/victastax/axlewatch-site/issues
