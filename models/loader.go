package models

import (
	"math"

	"github.com/GlenKelley/go-collada"
	glm "github.com/Jragonmiris/mathgl"
)

type Model struct {
	Name          string
	Transform     glm.Mat4d
	BaseTransform glm.Mat4d
	Geometry      []*Geometry
	Children      []*Model
	Parent        *Model
	Unit          float64
}

type Geometry struct {
	Name      string
	Triangles *Triangles
}

func (model *Model) WorldTransform() glm.Mat4d {
	t := model.Transform
	for parent := model.Parent; parent != nil; parent = parent.Parent {
		t = parent.Transform.Mul4(t)
	}
	return t
}

func NewSingleModel(name string, triangles *Triangles, transform glm.Mat4d) *Model {
	geometries := []*Geometry{NewGeometry(name+"-mesh", triangles)}
	model := NewModel(name, []*Model{}, geometries, transform)
	return model
}

func NewModel(name string, children []*Model, geometry []*Geometry, transform glm.Mat4d) *Model {
	model := &Model{
		name,
		transform,
		transform,
		geometry,
		children,
		nil,
		0.0,
	}
	for _, child := range children {
		child.Parent = model
	}
	return model
}

func EmptyModel(name string) *Model {
	return &Model{
		name,
		glm.Ident4d(),
		glm.Ident4d(),
		[]*Geometry{},
		[]*Model{},
		nil,
		0.0,
	}
}

func (model *Model) AddChild(child *Model) {
	child.Parent = model
	model.Children = append(model.Children, child)
}

func (model *Model) FindModelWithName(name string) (*Model, bool) {
	if model.Name == name {
		return model, true
	}
	for _, child := range model.Children {
		m, found := child.FindModelWithName(name)
		if found {
			return m, true
		}
	}
	return nil, false
}

func (model *Model) AddGeometry(geometry ...*Geometry) *Model {
	model.Geometry = append(model.Geometry, geometry...)
	return model
}

func (model *Model) SetTransform(m glm.Mat4d) *Model {
	model.Transform = m
	return model
}

func NewGeometry(name string, triangles *Triangles) *Geometry {
	return &Geometry{
		name,
		triangles,
	}
}

func RotationComponent(m glm.Mat4d) glm.Mat3d {
	m2 := glm.Ident3d()
	j := 0
	for i := 0; i < 3; i++ {
		v := glm.Vec4d{}
		v[i] = 1
		vt := m.Mul4x1(v)
		for k := 0; k < 3; k++ {
			m2[j] = vt[k]
			j++
		}
	}
	return m2
}

func Quaternion(m glm.Mat3d) glm.Quatd {
	q := glm.Quatd{}
	m00, m01, m02 := m[0], m[1], m[2]
	m10, m11, m12 := m[3], m[4], m[5]
	m20, m21, m22 := m[6], m[7], m[8]

	q.W = math.Sqrt(math.Max(0, 1+m00+m11+m22)) / 2
	q.V[0] = math.Copysign(math.Sqrt(math.Max(0, 1+m00-m11-m22))/2, m12-m21)
	q.V[1] = math.Copysign(math.Sqrt(math.Max(0, 1-m00+m11-m22))/2, m20-m02)
	q.V[2] = math.Copysign(math.Sqrt(math.Max(0, 1-m00-m11+m22))/2, m01-m10)
	return q
}

func LoadSceneAsModel(filename string) (*Model, error) {
	doc, err := collada.LoadDocument(filename)
	if err != nil {
		return nil, err
	}
	index, err := NewIndex(doc)
	if err != nil {
		return nil, err
	}
	model := EmptyModel("scene")
	model.Unit = doc.Asset.Unit.Meter
	switch doc.Asset.UpAxis {
	case collada.Xup:
	case collada.Yup:
	case collada.Zup:
		model.Transform = glm.HomogRotate3DXd(-90).Mul4(glm.HomogRotate3DZd(90))
	}

	geometryTemplates := make(map[collada.Id][]*Geometry)
	for id, mesh := range index.Mesh {
		geoms := make([]*Geometry, 0)
		for _, pl := range mesh.Triangles {
			geometry := NewGeometry(string(id), pl)
			geoms = append(geoms, geometry)
		}
		if len(geoms) > 0 {
			geometryTemplates[id] = geoms
		}
	}
	for _, node := range index.VisualScene.Node {
		child, ok := LoadModel(index, node, geometryTemplates)
		if ok {
			model.AddChild(child)
		}
	}
	return model, nil
}

func LoadModel(index *Index, node *collada.Node, geometryTemplates map[collada.Id][]*Geometry) (*Model, bool) {
	transform, ok := index.Transforms[node.Id]
	if !ok {
		transform = glm.Mat4d{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}
	}
	geoms := make([]*Geometry, 0)
	children := make([]*Model, 0)
	for _, geoinstance := range node.InstanceGeometry {
		geoid, _ := geoinstance.Url.Id()
		geoms = append(geoms, geometryTemplates[geoid]...)
	}
	for _, childNode := range node.Node {
		child, ok := LoadModel(index, childNode, geometryTemplates)
		if ok {
			children = append(children, child)
		}
	}
	model := NewModel(node.Name, children, geoms, transform)
	return model, len(geoms) > 0 || len(children) > 0
}

type Index struct {
	Collada     *collada.Collada
	Id          map[collada.Id]interface{}
	Data        map[collada.Id]interface{}
	Mesh        map[collada.Id]*Mesh
	Transforms  map[collada.Id]glm.Mat4d
	VisualScene *collada.VisualScene
	// animations: Object
	// cameras: Object
	// controllers: Object
	// effects: Object
	// geometries: Object
	// images: Object
	// lights: Object
	// materials: Object
	// scene: VisualScene
	// visualScenes: Object
}

type Mesh struct {
	VerticesId string
	Triangles  []*Triangles
}

type Triangles struct {
	VertexData []float64
	NormalData []float64
	Index      []int
}

func NewIndex(c *collada.Collada) (*Index, error) {
	index := &Index{
		c,
		make(map[collada.Id]interface{}),
		make(map[collada.Id]interface{}),
		make(map[collada.Id]*Mesh),
		make(map[collada.Id]glm.Mat4d),
		nil,
	}
	index.init()
	return index, nil
}

func (index *Index) AddId(id collada.Id, obj interface{}) {
	prev, ok := index.Id[id]
	if ok && prev != obj {
		panic("object id already defined: " + id)
	} else {
		index.Id[id] = obj
	}
}

func (index *Index) init() {
	index.indexVisualScenes()
	index.indexGeometry()

	ivs := index.Collada.Scene.InstanceVisualScene
	if ivs != nil {
		id, ok := ivs.Url.Id()
		if ok {
			index.VisualScene = index.Id[id].(*collada.VisualScene)
		}
	}
}

func NodeTransform(node *collada.Node) glm.Mat4d {
	transform := glm.Ident4d()
	for _, matrix := range node.Matrix {
		v := matrix.F()
		transform = transform.Mul4(glm.Mat4d{
			v[0], v[1], v[2], v[3],
			v[4], v[5], v[6], v[7],
			v[8], v[9], v[10], v[11],
			v[12], v[13], v[14], v[15],
		}.Transpose())
	}
	for _, translate := range node.Translate {
		v := translate.F()
		transform = transform.Mul4(glm.Translate3Dd(v[0], v[1], v[2]))
	}
	for _, rotation := range node.Rotate {
		v := rotation.F()
		transform = transform.Mul4(glm.HomogRotate3Dd(v[3]*math.Pi/180, glm.Vec3d{v[0], v[1], v[2]}))
	}
	for _, scale := range node.Scale {
		v := scale.F()
		transform = transform.Mul4(glm.Scale3Dd(v[0], v[1], v[2]))
	}
	return transform
}

func (index *Index) indexVisualScenes() {
	for _, lib := range index.Collada.LibraryVisualScenes {
		for _, vs := range lib.VisualScene {
			if len(vs.Id) != 0 {
				index.AddId(vs.Id, vs)
			}
			for _, node := range vs.Node {
				index.indexNode(node)
			}
		}
	}
}

func (index *Index) indexNode(node *collada.Node) {
	if len(node.Id) != 0 {
		index.AddId(node.Id, node)
		index.Transforms[node.Id] = NodeTransform(node)
	}
	for _, child := range node.Node {
		index.indexNode(child)
	}
}

func (index *Index) indexGeometry() {
	for _, lib := range index.Collada.LibraryGeometries {
		for _, g := range lib.Geometry {
			if len(g.Id) != 0 {
				index.AddId(g.Id, g)
			}
			index.Mesh[g.Id] = index.createMesh(g.Mesh)
		}
	}
}

func (index *Index) createMesh(m *collada.Mesh) *Mesh {
	for _, source := range m.Source {
		index.Data[source.Id] = source.FloatArray.F()
	}
	verticies := map[string]collada.Id{}
	for _, i := range m.Vertices.Input {
		verticies[i.Semantic], _ = i.Source.Id()
	}
	mesh := &Mesh{
		string(m.Vertices.Id),
		make([]*Triangles, len(m.Triangles)),
	}
	for k, pl := range m.Triangles {
		mpl := index.createMeshTriangles(pl, verticies)
		mesh.Triangles[k] = mpl
	}
	return mesh
}

type IndexPair struct {
	V int
	N int
}

func (index *Index) ReadOffsets(inputs []*collada.InputShared, verticies map[string]collada.Id) (int, []float64, int, []float64, int) {
	skip := len(inputs)
	v := 0
	n := 1
	var vs []float64 = nil
	var ns []float64 = nil
	for _, input := range inputs {
		if input.Semantic == "VERTEX" {
			vid := verticies["POSITION"]
			vs = index.Data[vid].([]float64)
			v = int(input.Offset)
		} else if input.Semantic == "NORMAL" {
			nid, _ := input.Source.Id()
			ns = index.Data[nid].([]float64)
			n = int(input.Offset)
		}
	}
	return v, vs, n, ns, skip
}

func (index *Index) createMeshTriangles(pl *collada.Triangles, verticies map[string]collada.Id) *Triangles {
	_, vs, _, ns, _ := index.ReadOffsets(pl.Input, verticies)
	mpl := Triangles{
		VertexData: vs,
		NormalData: ns,
		Index:      pl.P.I(),
	}
	return &mpl
}
