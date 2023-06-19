#include "MyMesh.h"
void MyMesh::GenerateCube(float a_fSize, vector3 a_v3Color)
{
	if (a_fSize < 0.01f)
		a_fSize = 0.01f;

	Release();
	Init();

	float fValue = a_fSize * 0.5f;
	//3--2
	//|  |
	//0--1

	vector3 point0(-fValue, -fValue, fValue); //0
	vector3 point1(fValue, -fValue, fValue); //1
	vector3 point2(fValue, fValue, fValue); //2
	vector3 point3(-fValue, fValue, fValue); //3

	vector3 point4(-fValue, -fValue, -fValue); //4
	vector3 point5(fValue, -fValue, -fValue); //5
	vector3 point6(fValue, fValue, -fValue); //6
	vector3 point7(-fValue, fValue, -fValue); //7

	//F
	AddQuad(point0, point1, point3, point2);

	//B
	AddQuad(point5, point4, point6, point7);

	//L
	AddQuad(point4, point0, point7, point3);

	//R
	AddQuad(point1, point5, point2, point6);

	//U
	AddQuad(point3, point2, point7, point6);

	//D
	AddQuad(point4, point5, point0, point1);

	// Adding information about color
	CompleteMesh(a_v3Color);
	CompileOpenGL3X();
}
void MyMesh::SubdivideTriangularMeshToRadius(vector3 t_v1, vector3 t_v2, vector3 t_v3, float radius, int steps)
{
	vector3 a_v1v2 = (t_v1 + t_v2) / 2;
	vector3 a_v2v3 = (t_v2 + t_v3) / 2;
	vector3 a_v3v1 = (t_v3 + t_v1) / 2;

	Normalize(a_v1v2);
	Normalize(a_v2v3);
	Normalize(a_v3v1);

	a_v1v2 = a_v1v2 * radius;
	a_v2v3 = a_v2v3 * radius;
	a_v3v1 = a_v3v1 * radius;

	if (steps == 1) {
		AddTri(t_v1, a_v3v1, a_v1v2);
		AddTri(t_v2, a_v1v2, a_v2v3);
		AddTri(a_v2v3, a_v3v1, t_v3);
		AddTri(a_v2v3, a_v1v2, a_v3v1);
	}
	else {
		SubdivideTriangularMeshToRadius(t_v1, a_v3v1, a_v1v2, radius, steps - 1);
		SubdivideTriangularMeshToRadius(t_v2, a_v1v2, a_v2v3, radius, steps - 1);
		SubdivideTriangularMeshToRadius(a_v2v3, a_v3v1, t_v3, radius, steps - 1);
		SubdivideTriangularMeshToRadius(a_v2v3, a_v1v2, a_v3v1, radius, steps - 1);
	}


}
void MyMesh::Normalize(vector3& vec)
{
	float length = sqrt(pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2));
	vec = vec / length;
}
void MyMesh::GenerateCone(float a_fRadius, float a_fHeight, int a_nSubdivisions, vector3 a_v3Color)
{
	if (a_fRadius < 0.01f)
		a_fRadius = 0.01f;

	if (a_fHeight < 0.01f)
		a_fHeight = 0.01f;

	if (a_nSubdivisions < 3)
		a_nSubdivisions = 3;
	if (a_nSubdivisions > 360)
		a_nSubdivisions = 360;

	Release();
	Init();



	//Generate 
	vector3 origin(0.0f, 0.0f, 0.0f);
	vector3 point1, point2;
	vector3 topPoint(0.0f, a_fHeight, 0.0f);

	float innerAngle = 2 * PI / a_nSubdivisions;

	for (int x = 0; x < a_nSubdivisions; x++) {

		point1 = vector3(
			a_fRadius * cos(x * innerAngle),
			0.0f,
			a_fRadius * sin(x * innerAngle)
		);
		point2 = vector3(
			a_fRadius * cos((x + 1) * innerAngle),
			0.0f,
			a_fRadius * sin((x + 1) * innerAngle)
		);
		//bottom piece
		AddTri(origin, point1, point2);
		//side face
		AddTri(point2, point1, topPoint);
	}

	// Adding information about color
	CompleteMesh(a_v3Color);
	CompileOpenGL3X();
}
void MyMesh::GenerateCylinder(float a_fRadius, float a_fHeight, int a_nSubdivisions, vector3 a_v3Color)
{
	if (a_fRadius < 0.01f)
		a_fRadius = 0.01f;

	if (a_fHeight < 0.01f)
		a_fHeight = 0.01f;

	if (a_nSubdivisions < 3)
		a_nSubdivisions = 3;
	if (a_nSubdivisions > 360)
		a_nSubdivisions = 360;

	Release();
	Init();

	//Generate 
	vector3 origin(0.0f, 0.0f, 0.0f);
	vector3 point1, point2;
	vector3 topPoint(0.0f, a_fHeight, 0.0f);

	float innerAngle = 2 * PI / a_nSubdivisions;

	for (int x = 0; x < a_nSubdivisions; x++) {

		point1 = vector3(
			a_fRadius * cos(x * innerAngle),
			0.0f,
			a_fRadius * sin(x * innerAngle)
		);
		point2 = vector3(
			a_fRadius * cos((x + 1) * innerAngle),
			0.0f,
			a_fRadius * sin((x + 1) * innerAngle)
		);


		vector3 topPoint1(point1.x, a_fHeight, point1.z);
		vector3 topPoint2(point2.x, a_fHeight, point2.z);
		vector3 topOrigin(0.0f, a_fHeight, 0.0f);
		//bottom piece
		AddTri(origin, point1, point2);

		//side face

		AddQuad(topPoint1, topPoint2, point1, point2);

		//top piece

		AddTri(topOrigin, topPoint2, topPoint1);

	}

	// Adding information about color
	CompleteMesh(a_v3Color);
	CompileOpenGL3X();
}
void MyMesh::GenerateTube(float a_fOuterRadius, float a_fInnerRadius, float a_fHeight, int a_nSubdivisions, vector3 a_v3Color)
{
	if (a_fOuterRadius < 0.01f)
		a_fOuterRadius = 0.01f;

	if (a_fInnerRadius < 0.005f)
		a_fInnerRadius = 0.005f;

	if (a_fInnerRadius > a_fOuterRadius)
		std::swap(a_fInnerRadius, a_fOuterRadius);

	if (a_fHeight < 0.01f)
		a_fHeight = 0.01f;

	if (a_nSubdivisions < 3)
		a_nSubdivisions = 3;
	if (a_nSubdivisions > 360)
		a_nSubdivisions = 360;

	Release();
	Init();

	vector3 origin(0.0f, 0.0f, 0.0f);
	vector3 outerPoint1, outerPoint2, innerPoint1, innerPoint2;
	vector3 topPoint(0.0f, a_fHeight, 0.0f);

	float innerAngle = 2 * PI / a_nSubdivisions;

	for (int x = 0; x < a_nSubdivisions; x++) {

		outerPoint1 = vector3(
			a_fOuterRadius * cos(x * innerAngle),
			0.0f,
			a_fOuterRadius * sin(x * innerAngle)
		);
		outerPoint2 = vector3(
			a_fOuterRadius * cos((x + 1) * innerAngle),
			0.0f,
			a_fOuterRadius * sin((x + 1) * innerAngle)
		);
		innerPoint1 = vector3(
			a_fInnerRadius * cos(x * innerAngle),
			0.0f,
			a_fInnerRadius * sin(x * innerAngle)
		);
		innerPoint2 = vector3(
			a_fInnerRadius * cos((x + 1) * innerAngle),
			0.0f,
			a_fInnerRadius * sin((x + 1) * innerAngle)
		);

		vector3 topOuterPoint1(outerPoint1.x, a_fHeight, outerPoint1.z);
		vector3 topOuterPoint2(outerPoint2.x, a_fHeight, outerPoint2.z);
		vector3 topInnerPoint1(innerPoint1.x, a_fHeight, innerPoint1.z);
		vector3 topInnerPoint2(innerPoint2.x, a_fHeight, innerPoint2.z);

		//outside face
		AddQuad(topOuterPoint1, topOuterPoint2, outerPoint1, outerPoint2);

		//inside face
		AddQuad(topInnerPoint2, topInnerPoint1, innerPoint2, innerPoint1);

		//top face
		AddQuad(topInnerPoint1, topInnerPoint2, topOuterPoint1, topOuterPoint2);

		//bottom face
		AddQuad(innerPoint2, innerPoint1, outerPoint2, outerPoint1);
	}

	// Adding information about color
	CompleteMesh(a_v3Color);
	CompileOpenGL3X();
}
void MyMesh::GenerateTorus(float a_fOuterRadius, float a_fInnerRadius, int a_nSubdivisionsA, int a_nSubdivisionsB, vector3 a_v3Color)
{
	if (a_fOuterRadius < 0.01f)
		a_fOuterRadius = 0.01f;

	if (a_fInnerRadius < 0.005f)
		a_fInnerRadius = 0.005f;

	if (a_fInnerRadius > a_fOuterRadius)
		std::swap(a_fInnerRadius, a_fOuterRadius);

	if (a_nSubdivisionsA < 3)
		a_nSubdivisionsA = 3;
	if (a_nSubdivisionsA > 360)
		a_nSubdivisionsA = 360;

	if (a_nSubdivisionsB < 3)
		a_nSubdivisionsB = 3;
	if (a_nSubdivisionsB > 360)
		a_nSubdivisionsB = 360;

	Release();
	Init();

	std::vector<vector3> vert;

	float radius = (a_fOuterRadius - a_fInnerRadius) / 2;

	for (int i = 0; i < a_nSubdivisionsA; i++) {
		for (int j = 0; j < a_nSubdivisionsB; j++) {
			float a_div1 = (float)j / a_nSubdivisionsB * PI * 2.0;
			float a_div2 = (float)i / a_nSubdivisionsA * PI * 2.0;
			float x = (a_fInnerRadius + radius * cos(a_div2)) * cos(a_div1);
			float y = (a_fInnerRadius + radius * cos(a_div2)) * sin(a_div1);
			float z = radius * sin(a_div2);
			vert.push_back(vector3(x, y, z));
		}
	}
	for (int i = 0; i < a_nSubdivisionsA; i++) {
		int i_next = (i + 1) % a_nSubdivisionsA;
		for (int j = 0; j < a_nSubdivisionsB; j++) {
			int l = (j + 1) % a_nSubdivisionsB;
			int a = i * a_nSubdivisionsB + j;
			int b = i * a_nSubdivisionsB + l;
			int c = i_next * a_nSubdivisionsB + l;
			int d = i_next * a_nSubdivisionsB + j;
			AddQuad(vert[a], vert[b], vert[d], vert[c]);
		}
	}

	// Adding information about color
	CompleteMesh(a_v3Color);
	CompileOpenGL3X();
}




void MyMesh::GenerateSphere(float a_fRadius, int a_nSubdivisions, vector3 a_v3Color)
{
	if (a_fRadius < 0.01f)
		a_fRadius = 0.01f;

	//Sets minimum and maximum of subdivisions
	if (a_nSubdivisions < 1)
	{
		GenerateCube(a_fRadius * 2.0f, a_v3Color);
		return;
	}
	if (a_nSubdivisions > 6)
		a_nSubdivisions = 6;

	Release();
	Init();


	float theta = (1 + sqrt(a_nSubdivisions)) / 2.0;
	
	float x = 1 / theta;
	//create initial verticies
	std::vector<vector3> vert;
	vert.push_back(vector3(0, x, -1));
	vert.push_back(vector3(x, 1, 0));
	vert.push_back(vector3(-x, 1, 0));
	vert.push_back(vector3(0, x, 1));
	vert.push_back(vector3(0, -x, 1));
	vert.push_back(vector3(-1, 0, x));
	vert.push_back(vector3(0, -x, -1));
	vert.push_back(vector3(1, 0, -x));
	vert.push_back(vector3(1, 0, x));
	vert.push_back(vector3(-1, 0, -x));
	vert.push_back(vector3(x, -1, 0));
	vert.push_back(vector3(-x, -1, 0));

	//normalize all vectors and project them onto the sphere of radius a_fRadius
	for (vector3& vec : vert) {
		Normalize(vec);
		vec = vec * a_fRadius;
	}

	int NumSubdivisions = 3;
	//subdivide the triangle formed by the 3 verticies into 4 pieces and project it onto the sphere of radius a_fRadius
	//do this 3 times
	//3 subdivisions creates a nice smoothe sphere and loaded instantly on my laptop
	SubdivideTriangularMeshToRadius(vert[2], vert[1], vert[0], a_fRadius, NumSubdivisions);
	SubdivideTriangularMeshToRadius(vert[1], vert[2], vert[3], a_fRadius, NumSubdivisions);
	SubdivideTriangularMeshToRadius(vert[5], vert[4], vert[3], a_fRadius, NumSubdivisions);
	SubdivideTriangularMeshToRadius(vert[4], vert[8], vert[3], a_fRadius, NumSubdivisions);
	SubdivideTriangularMeshToRadius(vert[7], vert[6], vert[0], a_fRadius, NumSubdivisions);
	SubdivideTriangularMeshToRadius(vert[6], vert[9], vert[0], a_fRadius, NumSubdivisions);
	SubdivideTriangularMeshToRadius(vert[11], vert[10], vert[4], a_fRadius, NumSubdivisions);
	SubdivideTriangularMeshToRadius(vert[10], vert[11], vert[6], a_fRadius, NumSubdivisions);
	SubdivideTriangularMeshToRadius(vert[9], vert[5], vert[2], a_fRadius, NumSubdivisions);
	SubdivideTriangularMeshToRadius(vert[5], vert[9], vert[11], a_fRadius, NumSubdivisions);
	SubdivideTriangularMeshToRadius(vert[8], vert[7], vert[1], a_fRadius, NumSubdivisions);
	SubdivideTriangularMeshToRadius(vert[7], vert[8], vert[10], a_fRadius, NumSubdivisions);
	SubdivideTriangularMeshToRadius(vert[2], vert[5], vert[3], a_fRadius, NumSubdivisions);
	SubdivideTriangularMeshToRadius(vert[8], vert[1], vert[3], a_fRadius, NumSubdivisions);
	SubdivideTriangularMeshToRadius(vert[9], vert[2], vert[0], a_fRadius, NumSubdivisions);
	SubdivideTriangularMeshToRadius(vert[1], vert[7], vert[0], a_fRadius, NumSubdivisions);
	SubdivideTriangularMeshToRadius(vert[11], vert[9], vert[6], a_fRadius, NumSubdivisions);
	SubdivideTriangularMeshToRadius(vert[7], vert[10], vert[6], a_fRadius, NumSubdivisions);
	SubdivideTriangularMeshToRadius(vert[5], vert[11], vert[4], a_fRadius, NumSubdivisions);
	SubdivideTriangularMeshToRadius(vert[10], vert[8], vert[4], a_fRadius, NumSubdivisions);



	// Adding information about color
	CompleteMesh(a_v3Color);
	CompileOpenGL3X();
}
void MyMesh::AddTri(vector3 a_vBottomLeft, vector3 a_vBottomRight, vector3 a_vTopLeft)
{
	//C
	//| \
	//A--B
	//This will make the triangle A->B->C 
	AddVertexPosition(a_vBottomLeft);
	AddVertexPosition(a_vBottomRight);
	AddVertexPosition(a_vTopLeft);
}
void MyMesh::AddQuad(vector3 a_vBottomLeft, vector3 a_vBottomRight, vector3 a_vTopLeft, vector3 a_vTopRight)
{
	//C--D
	//|  |
	//A--B
	//This will make the triangle A->B->C and then the triangle C->B->D
	AddVertexPosition(a_vBottomLeft);
	AddVertexPosition(a_vBottomRight);
	AddVertexPosition(a_vTopLeft);

	AddVertexPosition(a_vTopLeft);
	AddVertexPosition(a_vBottomRight);
	AddVertexPosition(a_vTopRight);
}
void MyMesh::Init(void)
{
	m_bBinded = false;
	m_uVertexCount = 0;

	m_VAO = 0;
	m_VBO = 0;

	glCullFace(GL_BACK);
	m_pShaderMngr = ShaderManager::GetInstance();
}
void MyMesh::Release(void)
{
	m_pShaderMngr = nullptr;

	if (m_VBO > 0)
		glDeleteBuffers(1, &m_VBO);

	if (m_VAO > 0)
		glDeleteVertexArrays(1, &m_VAO);

	m_lVertex.clear();
	m_lVertexPos.clear();
	m_lVertexCol.clear();
}
MyMesh::MyMesh()
{
	Init();
}
MyMesh::~MyMesh() { Release(); }
MyMesh::MyMesh(MyMesh& other)
{
	m_bBinded = other.m_bBinded;

	m_pShaderMngr = other.m_pShaderMngr;

	m_uVertexCount = other.m_uVertexCount;

	m_VAO = other.m_VAO;
	m_VBO = other.m_VBO;
}
MyMesh& MyMesh::operator=(MyMesh& other)
{
	if (this != &other)
	{
		Release();
		Init();
		MyMesh temp(other);
		Swap(temp);
	}
	return *this;
}
void MyMesh::Swap(MyMesh& other)
{
	std::swap(m_bBinded, other.m_bBinded);
	std::swap(m_uVertexCount, other.m_uVertexCount);

	std::swap(m_VAO, other.m_VAO);
	std::swap(m_VBO, other.m_VBO);

	std::swap(m_lVertex, other.m_lVertex);
	std::swap(m_lVertexPos, other.m_lVertexPos);
	std::swap(m_lVertexCol, other.m_lVertexCol);

	std::swap(m_pShaderMngr, other.m_pShaderMngr);
}
void MyMesh::CompleteMesh(vector3 a_v3Color)
{
	uint uColorCount = m_lVertexCol.size();
	for (uint i = uColorCount; i < m_uVertexCount; ++i)
	{
		m_lVertexCol.push_back(a_v3Color);
	}
}
void MyMesh::AddVertexPosition(vector3 a_v3Input)
{
	m_lVertexPos.push_back(a_v3Input);
	m_uVertexCount = m_lVertexPos.size();
}
void MyMesh::AddVertexColor(vector3 a_v3Input)
{
	m_lVertexCol.push_back(a_v3Input);
}
void MyMesh::CompileOpenGL3X(void)
{
	if (m_bBinded)
		return;

	if (m_uVertexCount == 0)
		return;

	CompleteMesh();

	for (uint i = 0; i < m_uVertexCount; i++)
	{
		//Position
		m_lVertex.push_back(m_lVertexPos[i]);
		//Color
		m_lVertex.push_back(m_lVertexCol[i]);
	}
	glGenVertexArrays(1, &m_VAO);//Generate vertex array object
	glGenBuffers(1, &m_VBO);//Generate Vertex Buffered Object

	glBindVertexArray(m_VAO);//Bind the VAO
	glBindBuffer(GL_ARRAY_BUFFER, m_VBO);//Bind the VBO
	glBufferData(GL_ARRAY_BUFFER, m_uVertexCount * 2 * sizeof(vector3), &m_lVertex[0], GL_STATIC_DRAW);//Generate space for the VBO

	// Position attribute
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(vector3), (GLvoid*)0);

	// Color attribute
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(vector3), (GLvoid*)(1 * sizeof(vector3)));

	m_bBinded = true;

	glBindVertexArray(0); // Unbind VAO
}
void MyMesh::Render(matrix4 a_mProjection, matrix4 a_mView, matrix4 a_mModel)
{
	// Use the buffer and shader
	GLuint nShader = m_pShaderMngr->GetShaderID("Basic");
	glUseProgram(nShader);

	//Bind the VAO of this object
	glBindVertexArray(m_VAO);

	// Get the GPU variables by their name and hook them to CPU variables
	GLuint MVP = glGetUniformLocation(nShader, "MVP");
	GLuint wire = glGetUniformLocation(nShader, "wire");

	//Final Projection of the Camera
	matrix4 m4MVP = a_mProjection * a_mView * a_mModel;
	glUniformMatrix4fv(MVP, 1, GL_FALSE, glm::value_ptr(m4MVP));

	//Solid
	glUniform3f(wire, -1.0f, -1.0f, -1.0f);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glDrawArrays(GL_TRIANGLES, 0, m_uVertexCount);

	//Wire
	glUniform3f(wire, 1.0f, 0.0f, 1.0f);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glEnable(GL_POLYGON_OFFSET_LINE);
	glPolygonOffset(-1.f, -1.f);
	glDrawArrays(GL_TRIANGLES, 0, m_uVertexCount);
	glDisable(GL_POLYGON_OFFSET_LINE);

	glBindVertexArray(0);// Unbind VAO so it does not get in the way of other objects
}
void MyMesh::GenerateCuboid(vector3 a_v3Dimensions, vector3 a_v3Color)
{
	Release();
	Init();

	vector3 v3Value = a_v3Dimensions * 0.5f;
	//3--2
	//|  |
	//0--1
	vector3 point0(-v3Value.x, -v3Value.y, v3Value.z); //0
	vector3 point1(v3Value.x, -v3Value.y, v3Value.z); //1
	vector3 point2(v3Value.x, v3Value.y, v3Value.z); //2
	vector3 point3(-v3Value.x, v3Value.y, v3Value.z); //3

	vector3 point4(-v3Value.x, -v3Value.y, -v3Value.z); //4
	vector3 point5(v3Value.x, -v3Value.y, -v3Value.z); //5
	vector3 point6(v3Value.x, v3Value.y, -v3Value.z); //6
	vector3 point7(-v3Value.x, v3Value.y, -v3Value.z); //7

	//F
	AddQuad(point0, point1, point3, point2);

	//B
	AddQuad(point5, point4, point6, point7);

	//L
	AddQuad(point4, point0, point7, point3);

	//R
	AddQuad(point1, point5, point2, point6);

	//U
	AddQuad(point3, point2, point7, point6);

	//D
	AddQuad(point4, point5, point0, point1);

	// Adding information about color
	CompleteMesh(a_v3Color);
	CompileOpenGL3X();
}