package main

import (
	"encoding/json"
	"fmt"
	"github.com/slobdell/basicMatrix"
	"io/ioutil"
	"math"
)

type Degrees float64
type Radians float64

const EARTH_RADIUS = 6371 * 1000.0 // meters

const FOUR_SPACES = "    "
const ACTUAL_GRAVITY = 9.80665

func panicForError(e error) {
	if e != nil {
		panic(e)
	}
}

type GeoPoint struct {
	Latitude  float64
	Longitude float64
}

func RadiansToDegrees(radians Radians) Degrees {
	return Degrees(radians * 180.0 / math.Pi)
}

func GetPointAhead(fromCoordinate GeoPoint, distanceMeters float64, azimuth Degrees) GeoPoint {
	radiusFraction := float64(distanceMeters / EARTH_RADIUS)

	bearing := float64(DegreesToRadians(azimuth))

	lat1 := geoAngle(fromCoordinate.Latitude)
	lng1 := geoAngle(fromCoordinate.Longitude)

	lat2_part1 := math.Sin(lat1) * math.Cos(radiusFraction)
	lat2_part2 := math.Cos(lat1) * math.Sin(radiusFraction) * math.Cos(bearing)

	lat2 := math.Asin(lat2_part1 + lat2_part2)

	lng2_part1 := math.Sin(bearing) * math.Sin(radiusFraction) * math.Cos(lat1)
	lng2_part2 := math.Cos(radiusFraction) - (math.Sin(lat1) * math.Sin(lat2))

	lng2 := lng1 + math.Atan2(lng2_part1, lng2_part2)
	lng2 = math.Mod((lng2+3*math.Pi), (2*math.Pi)) - math.Pi

	return GeoPoint{
		Latitude:  float64(RadiansToDegrees(Radians(lat2))),
		Longitude: float64(RadiansToDegrees(Radians(lng2))),
	}
}
func PointPlusDistanceEast(fromCoordinate GeoPoint, distance float64) GeoPoint {
	return GetPointAhead(fromCoordinate, distance, 90.0)
}

func PointPlusDistanceNorth(fromCoordinate GeoPoint, distance float64) GeoPoint {
	return GetPointAhead(fromCoordinate, distance, 0.0)
}
func MetersToGeopoint(latAsMeters, lonAsMeters float64) GeoPoint {
	point := GeoPoint{}
	pointEast := PointPlusDistanceEast(point, lonAsMeters)
	pointNorthEast := PointPlusDistanceNorth(pointEast, latAsMeters)
	return pointNorthEast
}

func DegreesToRadians(degrees Degrees) Radians {
	return Radians(degrees * math.Pi / 180.0)
}

func geoAngle(latOrLon float64) float64 {
	return float64(
		DegreesToRadians(
			Degrees(
				latOrLon,
			),
		),
	)
}
func GetDistanceMeters(fromCoordinate GeoPoint, toCoordinate GeoPoint) float64 {
	deltaLon := geoAngle(toCoordinate.Longitude - fromCoordinate.Longitude)
	deltaLat := geoAngle(toCoordinate.Latitude - fromCoordinate.Latitude)

	a := math.Pow(math.Sin(deltaLat/2.0), 2) +
		math.Cos(geoAngle(fromCoordinate.Latitude))*
			math.Cos(geoAngle(toCoordinate.Latitude))*
			math.Pow(math.Sin(deltaLon/2.0), 2)
	c := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1.0-a))
	return EARTH_RADIUS * c
}
func LatitudeToMeters(latitude float64) float64 {
	distance := GetDistanceMeters(
		GeoPoint{
			Latitude:  latitude,
			Longitude: 0.0,
		},
		GeoPoint{
			Latitude:  0.0,
			Longitude: 0.0,
		},
	)
	if latitude < 0 {
		distance *= -1
	}
	return distance
}

func LongitudeToMeters(longitude float64) float64 {
	distance := GetDistanceMeters(
		GeoPoint{
			Latitude:  0.0,
			Longitude: longitude,
		},
		GeoPoint{
			Latitude:  0.0,
			Longitude: 0.0,
		},
	)
	if longitude < 0 {
		distance *= -1
	}
	return distance
}

type sensorData struct {
	Timestamp     float64 `json:"timestamp"`
	GpsLat        float64 `json:"gps_lat"`
	GpsLon        float64 `json:"gps_lon"`
	GpsAlt        float64 `json:"gps_alt"`
	Pitch         float32 `json:"pitch"`
	Yaw           float32 `json:"yaw"`
	Roll          float32 `json:"roll"`
	AbsNorthAcc   float32 `json:"abs_north_acc"`
	AbsEastAcc    float32 `json:"abs_east_acc"`
	AbsUpAcc      float32 `json:"abs_up_acc"`
	VelNorth      float64 `json:"vel_north"`
	VelEast       float64 `json:"vel_east"`
	VelDown       float64 `json:"vel_down"`
	VelError      float64 `json:"vel_error"`
	AltitudeError float64 `json:"altitude_error"`
}

type outputPacket struct {
	sensorData
	PredictedLat float64 `json:"predicted_lat"`
	PredictedLon float64 `json:"predicted_lon"`
	PredictedAlt float64 `json:"predicted_alt"`
	ResultantMPH float64 `json:"resultant_mph"`
	GPSLat       float64 `json:"gps_lat"`
	GPSLon       float64 `json:"gps_lon"`
}

type sensorDataCollection []sensorData
type outputCollection []outputPacket

func readFileAsJson(filename string, outputEntityAddress interface{}) {
	fileContents, err := ioutil.ReadFile(filename)
	panicForError(err)

	err = json.Unmarshal(fileContents, outputEntityAddress)
	panicForError(err)
}

func writeJsonSerializableToFile(jsonEntity interface{}, filename string) {
	serialized, err := json.MarshalIndent(jsonEntity, "", FOUR_SPACES)
	err = ioutil.WriteFile(
		filename,
		serialized,
		0644,
	)
	panicForError(err)
}

/*
	Although these variables aren't expressive, they're based on existing mathematical conventions
	and in reality should be completely abstract.  The variables in my own words expressed below:

	H: For our usage, this should just be an identity matrix.  In practice this is meant to be
	a transformation matrix to standardize inputs to the system, but I'm enforcing this in the
	API itself; This should simplify usage and a bit of performance by not having to use this

	P: Newest estimate for average error for each part of state. This value will evolve internally
	from the kalman filter, so initializing as an identity matrix is also acceptable

	Q: Abstractly, the process error variance.  Explicitly for our use case, this is the covariance
	matrix for the accelerometer.  To find, you can leave the accelerometer at rest and take the standard
	deviation, then square that for the variance.  Matrix would then be

	[AVariance 0]
	[0 AVariance]

	Additionally, when computing standard deviation, in this context it would make sense to override
	the mean value of the readings to be 0 to account for a blatant offset from the sensor.

	R: Abstractly, the measurement error variance. Explicitly for our use case, this is the covariance
	matrix of the GPS.  If you can get the actual standard deviation of the GPS, this might work, but
	if you take GPS readings at rest, you might have a GPS lock that results in extremely minimal error.

	In practice, I just took the advertised +/- value from the GPS (i.e. uBlock is accurate +/- 1 meter allegedly, so you can use that).

	u: Overridden during each prediction step; Setting as a struct attribute for performance reasons. This
	is the input matrix of high frequency sensor readings that without subject to any error would give us
	an accurate state of the world.

	In our case, it's a 1x1 matrix of accelerometer input in a given direction.

	z: Overridden during each prediction step; Setting as a struct attribute for performance reasongs. This
	is the input matrix of low frequency sensor readings that are absolute but presumably high standard
	deviation.

	In our case, it's a 2x1 matrix of GPS position and velocity
	[ P
	  v ]

	  A: The state transition matrix. Abstractly, this is a matrix that defines a set of of equations that define what the next step would like given no additional inputs but a "next step" (or more than likely, change in time). Given that this struct is explicitly for fusing position and acceleration, it's:

	  [ 1 t
	    0 1 ]

	To explain the above, if you have position, then its next position is the previous position + current velocity * times. If you have velocity, then its next velocity will be the current velocity.

	B: Control matrix. Given input changes to the system, this matrix multiplied by the input will present new deltas
	to the current state.  In our case, these are the equations needed to handle input acceleration.  Specifically:

	[ 0.5t^2
	  t     ]
*/
type KalmanFilterFusedPositionAccelerometer struct {
	I                            *basicMatrix.Matrix // identity matrix used in some calculations
	H                            *basicMatrix.Matrix // transformation matrix for input data
	P                            *basicMatrix.Matrix // initial guess for covariance
	Q                            *basicMatrix.Matrix // process (accelerometer) error variance
	R                            *basicMatrix.Matrix // measurement (GPS) error variance
	u                            *basicMatrix.Matrix // INPUT control (accelerometer) matrix
	z                            *basicMatrix.Matrix // INPUT measurement (GPS) matrix
	A                            *basicMatrix.Matrix // State Transition matrix
	B                            *basicMatrix.Matrix // Control matrix
	currentState                 *basicMatrix.Matrix
	currentStateTimestampSeconds float64
}

func (k *KalmanFilterFusedPositionAccelerometer) Predict(accelerationThisAxis, timestampNow float64) {
	deltaT := timestampNow - k.currentStateTimestampSeconds

	k.recreateControlMatrix(deltaT)
	k.recreateStateTransitionMatrix(deltaT)

	k.u.Put(0, 0, accelerationThisAxis)

	k.currentState = (k.A.MultipliedBy(k.currentState)).Add(k.B.MultipliedBy(k.u))

	k.P = ((k.A.MultipliedBy(k.P)).MultipliedBy(k.A.Transpose())).Add(k.Q)

	k.currentStateTimestampSeconds = timestampNow
}

func (k *KalmanFilterFusedPositionAccelerometer) Update(position float64, velocityThisAxis float64, positionError *float64, velocityError float64) {

	k.z.Put(0, 0, float64(position))
	k.z.Put(1, 0, float64(velocityThisAxis))

	if positionError != nil {
		k.R.Put(0, 0, *positionError**positionError)
	} else {
	}
	k.R.Put(1, 1, velocityError*velocityError)

	y := k.z.Subtract(k.currentState)
	s := k.P.Add(k.R)
	sInverse, err := s.Inverse()
	if err != nil {
		// matrix has no inverse, abort
		return
	}
	K := k.P.MultipliedBy(sInverse)

	k.currentState = k.currentState.Add(K.MultipliedBy(y))

	k.P = (k.I.Subtract(K)).MultipliedBy(k.P)

	/*
		above is equivalent to:
			updatedP := k.P.Subtract(K.MultipliedBy(k.P))
		which would explain some confusion on the internets
	*/
}

func (k *KalmanFilterFusedPositionAccelerometer) recreateControlMatrix(deltaSeconds float64) {
	dtSquared := 0.5 * deltaSeconds * deltaSeconds

	k.B.Put(0, 0, dtSquared)
	k.B.Put(1, 0, deltaSeconds)
}
func (k *KalmanFilterFusedPositionAccelerometer) recreateStateTransitionMatrix(deltaSeconds float64) {
	k.A.Put(0, 0, 1.0)
	k.A.Put(0, 1, deltaSeconds)

	k.A.Put(1, 0, 0.0)
	k.A.Put(1, 1, 1.0)
}

func (k *KalmanFilterFusedPositionAccelerometer) GetPredictedPosition() float64 {
	return (k.currentState.Get(0, 0))
}

func (k *KalmanFilterFusedPositionAccelerometer) GetPredictedVelocityThisAxis() float64 {
	return (k.currentState.Get(1, 0))
}

func NewKalmanFilterFusedPositionAccelerometer(initialPosition float64,
	initialVelocity float64, // TODO unused still
	positionStandardDeviation float64,
	accelerometerStandardDeviation float64,
	currentTimestampSeconds float64) *KalmanFilterFusedPositionAccelerometer {

	currentState := basicMatrix.NewMatrix(2, 1)

	currentState.Put(0, 0, float64(initialPosition))
	currentState.Put(1, 0, float64(initialVelocity))

	u := basicMatrix.NewMatrix(1, 1)
	z := basicMatrix.NewMatrix(2, 1)
	H := basicMatrix.NewIdentityMatrix(2, 2)
	P := basicMatrix.NewIdentityMatrix(2, 2)
	I := basicMatrix.NewIdentityMatrix(2, 2)

	Q := basicMatrix.NewMatrix(2, 2)
	Q.Put(0, 0, accelerometerStandardDeviation*accelerometerStandardDeviation)
	Q.Put(1, 1, accelerometerStandardDeviation*accelerometerStandardDeviation)

	R := basicMatrix.NewMatrix(2, 2)
	R.Put(0, 0, float64(positionStandardDeviation*positionStandardDeviation))
	// TODO might need to play with this value
	R.Put(1, 1, float64(positionStandardDeviation*positionStandardDeviation))
	B := basicMatrix.NewMatrix(2, 1)
	A := basicMatrix.NewMatrix(2, 2)

	return &KalmanFilterFusedPositionAccelerometer{
		I:                            I,
		A:                            A,
		B:                            B,
		z:                            z,
		u:                            u,
		H:                            H,
		P:                            P,
		Q:                            Q,
		R:                            R,
		currentState:                 currentState,
		currentStateTimestampSeconds: currentTimestampSeconds,
	}
}

func main() {
	var collection sensorDataCollection
	readFileAsJson("pos_final.json", &collection)

	initialSensorData := collection[0]

	latLonStandardDeviation := 2.0 // +/- 1m, increased for safety
	altitudeStandardDeviation := 3.518522417151836

	// got this value by getting standard deviation from accelerometer, assuming that mean SHOULD be 0
	accelerometerEastStandardDeviation := ACTUAL_GRAVITY * 0.033436506994600976
	accelerometerNorthStandardDeviation := ACTUAL_GRAVITY * 0.05355371135598354
	accelerometerUpStandardDeviation := ACTUAL_GRAVITY * 0.2088683796078286

	longitudeEastKalmanFilter := NewKalmanFilterFusedPositionAccelerometer(
		LongitudeToMeters(initialSensorData.GpsLon),
		initialSensorData.VelEast,
		latLonStandardDeviation,
		accelerometerEastStandardDeviation,
		initialSensorData.Timestamp,
	)
	latitudeNorthKalmanFilter := NewKalmanFilterFusedPositionAccelerometer(
		LatitudeToMeters(initialSensorData.GpsLat),
		initialSensorData.VelNorth,
		latLonStandardDeviation,
		accelerometerNorthStandardDeviation,
		initialSensorData.Timestamp,
	)
	altitudeUpKalmanFilter := NewKalmanFilterFusedPositionAccelerometer(
		initialSensorData.GpsAlt,
		initialSensorData.VelDown*-1.0,
		altitudeStandardDeviation,
		accelerometerUpStandardDeviation,
		initialSensorData.Timestamp,
	)

	outputs := make(outputCollection, 0)

	for i := 1; i < len(collection); i++ {
		data := collection[i]

		longitudeEastKalmanFilter.Predict(
			float64(data.AbsEastAcc)*ACTUAL_GRAVITY,
			data.Timestamp,
		)
		latitudeNorthKalmanFilter.Predict(
			float64(data.AbsNorthAcc)*ACTUAL_GRAVITY,
			data.Timestamp,
		)
		altitudeUpKalmanFilter.Predict(
			float64(data.AbsUpAcc)*ACTUAL_GRAVITY,
			data.Timestamp,
		)

		if data.GpsLat != 0.0 {

			var defaultPositionErr *float64 = nil
			vEast := data.VelEast
			longitudeAsMeters := LongitudeToMeters(data.GpsLon)
			longitudeEastKalmanFilter.Update(
				longitudeAsMeters,
				vEast,
				defaultPositionErr,
				data.VelError,
			)

			vNorth := data.VelNorth
			latitudeAsMeters := LatitudeToMeters(data.GpsLat)
			latitudeNorthKalmanFilter.Update(
				latitudeAsMeters,
				vNorth,
				defaultPositionErr,
				data.VelError,
			)

			vUp := data.VelDown * -1.0
			altitudeUpKalmanFilter.Update(
				data.GpsAlt,
				vUp,
				&data.AltitudeError,
				data.VelError,
			)
		}
		predictedLonMeters := longitudeEastKalmanFilter.GetPredictedPosition()
		predictedLatMeters := latitudeNorthKalmanFilter.GetPredictedPosition()
		predictedAlt := altitudeUpKalmanFilter.GetPredictedPosition()

		point := MetersToGeopoint(
			predictedLatMeters,
			predictedLonMeters,
		)
		predictedLon := point.Longitude
		predictedLat := point.Latitude

		predictedVE := longitudeEastKalmanFilter.GetPredictedVelocityThisAxis()
		predictedVN := latitudeNorthKalmanFilter.GetPredictedVelocityThisAxis()

		resultantV := math.Sqrt(math.Pow(predictedVE, 2) + math.Pow(predictedVN, 2))

		deltaT := data.Timestamp - initialSensorData.Timestamp
		fmt.Printf("%f seconds in, Lat: %f, Lon: %f, Alt: %f, V(mph): %f, A: %f\n", deltaT, predictedLat, predictedLon, predictedAlt, 2.23694*resultantV, float64(data.AbsEastAcc)*ACTUAL_GRAVITY)
		outputs = append(
			outputs,
			outputPacket{
				sensorData:   data,
				PredictedLat: predictedLat,
				PredictedLon: predictedLon,
				PredictedAlt: predictedAlt,
				ResultantMPH: 2.23694 * resultantV,
				GPSLat:       data.GpsLat,
				GPSLon:       data.GpsLon,
			},
		)
	}
	fmt.Printf("got to end with no crash: %s\n", longitudeEastKalmanFilter)
	writeJsonSerializableToFile(outputs, "finalOut.json")
}
