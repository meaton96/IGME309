#include "Octant.h"
using namespace BTX;
//  Octant
uint Octant::m_uOctantCount = 0;
uint Octant::m_uMaxLevel = 3;
uint Octant::m_uIdealEntityCount = 5;
Octant::Octant(uint a_nMaxLevel, uint a_nIdealEntityCount)
{
	Init(); // Init the default values

	m_uOctantCount = 0;
	m_uMaxLevel = a_nMaxLevel;
	m_uIdealEntityCount = a_nIdealEntityCount;
	m_uID = m_uOctantCount;
	m_pRoot = this;
	m_lChild.clear();

	// Get a list of all entities in the world
	uint entityCount = m_pEntityMngr->GetEntityCount();

	if (entityCount == 0)
		return;

	std::vector<vector3> lMinMax;
	for (uint i = 0; i < entityCount; i++)
	{
		RigidBody* pRigidBody = m_pEntityMngr->GetEntity(i)->GetRigidBody();
		vector3 min = pRigidBody->GetMinGlobal();
		vector3 max = pRigidBody->GetMaxGlobal();
		lMinMax.push_back(min);
		lMinMax.push_back(max);
	}

	RigidBody pRigidBody = RigidBody(lMinMax);

	// Set the values of the octant
	m_fSize = pRigidBody.GetHalfWidth().x * 2.0f;
	m_v3Center = pRigidBody.GetCenterGlobal();
	m_v3Min = m_v3Center - pRigidBody.GetHalfWidth();
	m_v3Max = m_v3Center + pRigidBody.GetHalfWidth();

	m_uOctantCount++; // When we add an octant we increment the count
	

	ConstructTree(m_uMaxLevel); // Construct the children
}


bool Octant::IsColliding(uint a_uRBIndex)
{
	/// Get how many objects there are in the world
	uint entityCount = m_pEntityMngr->GetEntityCount();
	// If the index given is larger than the number of elements in the bounding object there is no collision
	if (a_uRBIndex >= entityCount)
		return false;
	// Get bounding box of the entity at given index
	RigidBody* pRB = m_pEntityMngr->GetEntity(a_uRBIndex)->GetRigidBody();
	vector3 v3MinG = pRB->GetMinGlobal();
	vector3 v3MaxG = pRB->GetMaxGlobal();

	if (m_v3Max.x < v3MinG.x || m_v3Min.x > v3MaxG.x)
		return false;
	if (m_v3Max.y < v3MinG.y || m_v3Min.y > v3MaxG.y)
		return false;
	if (m_v3Max.z < v3MinG.z || m_v3Min.z > v3MaxG.z)
		return false;
	return true;
}
void Octant::Display(uint a_nIndex, vector3 a_v3Color)
{
	if (a_nIndex >= m_uChildren)
		return;
	// Render a cube for the child at the specified index
	m_pModelMngr->AddWireCubeToRenderList(glm::translate(IDENTITY_M4, m_pChild[a_nIndex]->m_v3Center) *
		glm::scale(vector3(m_pChild[a_nIndex]->m_fSize)), a_v3Color);
}
void Octant::Display(vector3 a_v3Color)
{
	//this is meant to be a recursive method, in starter code will only display the root
	//even if other objects are created
	
	if (m_uChildren == 0) {
		m_pModelMngr->AddWireCubeToRenderList(glm::translate(IDENTITY_M4, m_v3Center) *
			glm::scale(vector3(m_fSize)), a_v3Color);
	}
	else {
		for (uint x = 0; x < m_uChildren; x++) {
			m_pChild[x]->Display(a_v3Color);
		}
	}
}
void Octant::Subdivide(void)
{
	// If this node has reached the maximum depth return without changes
	if (m_uLevel >= m_uMaxLevel)
		return;

	// If this node has been already subdivided return without changes
	if (m_uChildren != 0)
		return;

	// Calculate new size for children octants
	float fSize = m_fSize / 2.0f;
	float fSizeHalf = fSize / 2.0f;
	vector3 v3Center;
	
	// Create 8 children
	for (int i = 0; i < 8; i++)
	{
		v3Center = m_v3Center;
		v3Center.x += ((i & 1) ? fSizeHalf : -fSizeHalf);  // +x for odd indices, -x for even
		v3Center.y += ((i & 2) ? fSizeHalf : -fSizeHalf);  // +y for indices 2,3,6,7, -y for others
		v3Center.z += ((i & 4) ? fSizeHalf : -fSizeHalf);  // +z for indices 4,5,6,7, -z for others
		m_pChild[i] = new Octant(v3Center, fSize);

		m_pChild[i]->m_uMaxLevel = m_uMaxLevel;
		m_pChild[i]->m_uIdealEntityCount = m_uIdealEntityCount;
		m_pChild[i]->m_pRoot = m_pRoot;

		m_pChild[i]->m_uLevel = m_uLevel + 1;

		if (m_pChild[i]->m_uLevel < m_uMaxLevel) m_pChild[i]->Subdivide();
	}

	// Increment the children count
	m_uChildren = 8;
}
bool Octant::ContainsAtLeast(uint a_nEntities)
{
	uint count = 0;
	uint entityCount = m_pEntityMngr->GetEntityCount();

	for (uint i = 0; i < entityCount; i++)
	{
		if (IsColliding(i))
			count++;
	}
	
	return (count >= a_nEntities);
}
void Octant::AssignIDtoEntity(void)
{
	// Recursive method
	// Have to traverse the tree and make sure to tell the entity manager
	// what octant (space) each object is at

	for (uint nChild = 0; nChild < m_uChildren; nChild++)
	{
		m_pChild[nChild]->AssignIDtoEntity();
	}

	if (m_uChildren == 0) // We're in a leaf
	{
		uint entityCount = m_pEntityMngr->GetEntityCount();
		for (uint i = 0; i < entityCount; i++)
		{
			if (IsColliding(i))
			{
				m_pEntityMngr->AddDimension(i, m_uID);
				m_EntityList.push_back(i);
			}
		}
	}
}
//-------------------------------------------------------------------------------------------------------------------
// You can assume the following is fine and does not need changes, you may add onto it but the code is fine as is
// in the proposed solution.
void Octant::Init(void)
{
	m_uChildren = 0;

	m_fSize = 0.0f;

	m_uID = m_uOctantCount;
	m_uLevel = 0;

	m_v3Center = vector3(0.0f);
	m_v3Min = vector3(0.0f);
	m_v3Max = vector3(0.0f);

	m_pModelMngr = ModelManager::GetInstance();
	m_pEntityMngr = EntityManager::GetInstance();
	m_pRoot = nullptr;
	m_pParent = nullptr;
	for (uint n = 0; n < 8; n++)
	{
		m_pChild[n] = nullptr;
	}
}
void Octant::Swap(Octant& other)
{
	std::swap(m_uChildren, other.m_uChildren);

	std::swap(m_fSize, other.m_fSize);
	std::swap(m_uID, other.m_uID);
	std::swap(m_pRoot, other.m_pRoot);
	std::swap(m_lChild, other.m_lChild);

	std::swap(m_v3Center, other.m_v3Center);
	std::swap(m_v3Min, other.m_v3Min);
	std::swap(m_v3Max, other.m_v3Max);

	m_pModelMngr = ModelManager::GetInstance();
	m_pEntityMngr = EntityManager::GetInstance();

	std::swap(m_uLevel, other.m_uLevel);
	std::swap(m_pParent, other.m_pParent);
	for (uint i = 0; i < 8; i++)
	{
		std::swap(m_pChild[i], other.m_pChild[i]);
	}
}
void Octant::Release(void)
{
	//this is a special kind of release, it will only happen for the root
	if (m_uLevel == 0)
	{
		KillBranches();
	}
	m_uChildren = 0;
	m_fSize = 0.0f;
	m_EntityList.clear();
	m_lChild.clear();
}
void Octant::ConstructTree(uint a_nMaxLevel)
{
	//If this method is tried to be applied to something else
	//other than the root, don't.
	if (m_uLevel != 0)
		return;

	m_uMaxLevel = a_nMaxLevel; //Make sure you mark which is the maximum level you are willing to reach
	m_uOctantCount = 1;// We will always start with one octant

	//If this was initialized before make sure its empty
	m_EntityList.clear();//Make sure the list of entities inside this octant is empty
	KillBranches();
	m_lChild.clear();

	//If we have more entities than those that we ideally want in this octant we subdivide it
	if (ContainsAtLeast(m_uIdealEntityCount))
	{
		Subdivide();
	}
	AssignIDtoEntity();//Add octant ID to Entities
	ConstructList();//construct the list of objects
}
//The big 3
Octant::Octant(vector3 a_v3Center, float a_fSize)
{
	//Will create the octant object based on the center and size but will not construct children
	Init();
	m_v3Center = a_v3Center;
	m_fSize = a_fSize;

	m_v3Min = m_v3Center - (vector3(m_fSize) / 2.0f);
	m_v3Max = m_v3Center + (vector3(m_fSize) / 2.0f);

	m_uOctantCount++;
}
Octant::Octant(Octant const& other)
{
	m_uChildren = other.m_uChildren;
	m_v3Center = other.m_v3Center;
	m_v3Min = other.m_v3Min;
	m_v3Max = other.m_v3Max;

	m_fSize = other.m_fSize;
	m_uID = other.m_uID;
	m_uLevel = other.m_uLevel;
	m_pParent = other.m_pParent;

	m_pRoot, other.m_pRoot;
	m_lChild, other.m_lChild;

	m_pModelMngr = ModelManager::GetInstance();
	m_pEntityMngr = EntityManager::GetInstance();

	for (uint i = 0; i < 8; i++)
	{
		m_pChild[i] = other.m_pChild[i];
	}
}
Octant& Octant::operator=(Octant const& other)
{
	if (this != &other)
	{
		Release();
		Init();
		Octant temp(other);
		Swap(temp);
	}
	return *this;
}
Octant::~Octant() { Release(); };
//Accessors
float Octant::GetSize(void) { return m_fSize; }
vector3 Octant::GetCenterGlobal(void) { return m_v3Center; }
vector3 Octant::GetMinGlobal(void) { return m_v3Min; }
vector3 Octant::GetMaxGlobal(void) { return m_v3Max; }
uint Octant::GetOctantCount(void) { return m_uOctantCount; }
bool Octant::IsLeaf(void) { return m_uChildren == 0; }
Octant* Octant::GetParent(void) { return m_pParent; }
//--- other methods
Octant * Octant::GetChild(uint a_nChild)
{
	//if its asking for more than the 8th children return nullptr, as we don't have one
	if (a_nChild > 7) return nullptr;
	return m_pChild[a_nChild];
}
void Octant::KillBranches(void)
{
	/*This method has recursive behavior that is somewhat hard to explain line by line but
	it will traverse the whole tree until it reaches a node with no children and
	once it returns from it to its parent it will kill all of its children, recursively until
	it reaches back the node that called it.*/
	for (uint nIndex = 0; nIndex < m_uChildren; nIndex++)
	{
		m_pChild[nIndex]->KillBranches();
		delete m_pChild[nIndex];
		m_pChild[nIndex] = nullptr;
	}
	m_uChildren = 0;
}
void Octant::DisplayLeaves(vector3 a_v3Color)
{
	/*
	* Recursive method
	* it will traverse the tree until it find leaves and once found will draw them
	*/
	uint nLeafs = m_lChild.size(); //get how many children this will have (either 0 or 8)
	for (uint nChild = 0; nChild < nLeafs; nChild++)
	{
		m_lChild[nChild]->DisplayLeaves(a_v3Color);
	}
	//Draw the cube
	m_pModelMngr->AddWireCubeToRenderList(glm::translate(IDENTITY_M4, m_v3Center) *
		glm::scale(vector3(m_fSize)), a_v3Color);
}
void Octant::ClearEntityList(void)
{
	/*
	* Recursive method
	* It will traverse the tree until it find leaves and once found will clear its content
	*/
	for (uint nChild = 0; nChild < m_uChildren; nChild++)
	{
		m_pChild[nChild]->ClearEntityList();
	}
	m_EntityList.clear();
}
void Octant::ConstructList(void)
{
	/*
	* Recursive method
	* It will traverse the tree adding children
	*/
	for (uint nChild = 0; nChild < m_uChildren; nChild++)
	{
		m_pChild[nChild]->ConstructList();
	}

	//if we find a non-empty child add it to the tree
	if (m_EntityList.size() > 0)
	{
		m_pRoot->m_lChild.push_back(this);
	}
}
