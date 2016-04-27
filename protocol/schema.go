package protocol

// Profile ...
type Profile struct {
	BodyDensity  float64   // default 0.2
	BodyBox      []float64 // { width, height, length }
	BodyZOffset  float64   // Offset Adjust from center of wheels
	Wheelbase    float64   // default 0.267m
	Tread        float64   // default 0.160m
	TireDensity  float64   // default 0.1
	TireDiameter float64   // default 0.088m
	TireWidth    float64   // default 0.033m
}

// Input ...
type Input struct {
	Name     string  `json:"name"`
	Steering float64 `json:"steering"`
	Accel    float64 `json:"accel"`
	Brake    float64 `json:"brake"`
}

// Attitude ...
type Attitude struct {
	Position   []float64 `json:"position"`   // length=3
	Quaternion []float64 `json:"quaternion"` // length=4
}

// Vehicle ...
type Vehicle struct {
	Name  string
	Body  Attitude   `json:"body"`
	Tires []Attitude `json:"tires"`
}

// Output ...
type Output struct {
	Self   Vehicle
	Others []Vehicle
}
