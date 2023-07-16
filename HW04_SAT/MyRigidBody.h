/*----------------------------------------------
Programmer: Alberto Bobadilla (labigm@rit.edu)
Date: 2021/02
Update: 2021/02
----------------------------------------------*/
#ifndef __MYRIGIDBODY_H_
#define __MYRIGIDBODY_H_

#include "BTX\BTX.h"
#include "BTXs\BTXs.h"

namespace BTX
{

//System Class
class MyRigidBody
{
	

	ModelManager* m_pModelMngr = nullptr; //for displaying the Rigid Body

	bool m_bVisibleBS = false; //Visibility of bounding sphere
	bool m_bVisibleOBB = true; //Visibility of Oriented bounding box
	bool m_bVisibleARBB = true; //Visibility of axis (Re)aligned bounding box

	float m_fRadius = 0.0f; //Radius

	vector3 m_v3ColorColliding = C_RED; //Color when colliding
	vector3 m_v3ColorNotColliding = C_WHITE; //Color when not colliding

	vector3 m_v3Center = ZERO_V3; //center point in local space
	vector3 m_v3MinL = ZERO_V3; //minimum coordinate in local space (for OBB)
	vector3 m_v3MaxL = ZERO_V3; //maximum coordinate in local space (for OBB)

	vector3 m_v3MinG = ZERO_V3; //minimum coordinate in global space (for ARBB)
	vector3 m_v3MaxG = ZERO_V3; //maximum coordinate in global space (for ARBB)

	vector3 m_v3HalfWidth = ZERO_V3; //half the size of the Oriented Bounding Box
	vector3 m_v3ARBBSize = ZERO_V3;// size of the Axis (Re)Alligned Bounding Box

	matrix4 m_m4ToWorld = IDENTITY_M4; //Matrix that will take us from local to world coordinate

	std::set<MyRigidBody*> m_CollidingRBSet; //set of rigid bodies this one is colliding with
	
	

	

	
	

public:
	/*
	Usage: Constructor
	Arguments: std::vector<vector3> a_pointList -> list of points to make the Rigid Body for
	Output: class object instance
	*/
	MyRigidBody(std::vector<vector3> a_pointList);
	/*
	Usage: Copy Constructor
	Arguments: class object to copy
	Output: class object instance
	*/
	MyRigidBody(MyRigidBody const& other);
	/*
	Usage: Copy Assignment Operator
	Arguments: class object to copy
	Output: ---
	*/
	MyRigidBody& operator=(MyRigidBody const& other);
	/*
	Usage: Destructor
	Arguments: ---
	Output: ---
	*/
	~MyRigidBody(void);
	/*
	Usage: Changes object contents for other object's
	Arguments: other -> object to swap content from
	Output: ---
	*/
	void Swap(MyRigidBody& other);

	/*
	USAGE: Will render the Rigid Body at the model to world matrix
	ARGUMENTS: ---
	OUTPUT: ---
	*/
	void AddToRenderList(void);

	/*
	USAGE: Clears the colliding list
	ARGUMENTS: ---
	OUTPUT: ---
	*/
	void ClearCollidingList(void);

	/*
	USAGE: Mark collision with the incoming Rigid Body
	ARGUMENTS: MyRigidBody* other -> inspected rigid body
	OUTPUT: ---
	*/
	void AddCollisionWith(MyRigidBody* a_pOther);

	/*
	USAGE: Remove marked collision with the incoming Rigid Body
	ARGUMENTS: MyRigidBody* other -> inspected rigid body
	OUTPUT: ---
	*/
	void RemoveCollisionWith(MyRigidBody* a_pOther);

	/*
	USAGE: Tells if the object is colliding with the incoming one
	ARGUMENTS: MyRigidBody* const other -> inspected rigid body
	OUTPUT: are they colliding?
	*/
	bool IsColliding(MyRigidBody* const a_pOther);
#pragma region Accessors
	/*
	Usage: Gets visibility of bounding sphere
	Arguments: ---
	Output: visibility
	*/
	bool GetVisibleBS(void);
	/*
	Usage: Sets visibility of bounding sphere
	Arguments: bool a_bVisibility -> visibility to set
	Output: ---
	*/
	void SetVisibleBS(bool a_bVisibility);
	/*
	Usage: Gets visibility of oriented bounding box
	Arguments: ---
	Output: visibility
	*/
	bool GetVisibleOBB(void);
	/*
	Usage: Sets visibility of oriented bounding box
	Arguments: bool a_bVisibility -> visibility to set
	Output: ---
	*/
	void SetVisibleOBB(bool a_bVisibility);
	/*
	Usage: Gets visibility of axis (re)aligned bounding box
	Arguments: ---
	Output: visibility
	*/
	bool GetVisibleARBB(void);
	/*
	Usage: Sets visibility of axis (re)aligned bounding box
	Arguments: bool a_bVisibility -> visibility to set
	Output: ---
	*/
	void SetVisibleARBB(bool a_bVisibility);
	/*
	Usage: Gets radius
	Arguments: ---
	Output: radius
	*/
	float GetRadius(void);
	/*
	Usage: Gets the color when colliding
	Arguments: ---
	Output: color
	*/
	vector3 GetColorColliding(void);
	/*
	Usage: Sets the color when colliding
	Arguments: vector3 a_v3Color -> color
	Output: ---
	*/
	void SetColorColliding(vector3 a_v3Color);
	/*
	Usage: Gets the color when not colliding
	Arguments: ---
	Output: color
	*/
	vector3 GetColorNotColliding(void);
	/*
	Usage: Sets the color when colliding
	Arguments: vector3 a_v3Color -> color
	Output: ---
	*/
	void SetColorNotColliding(vector3 a_v3Color);
	/*
	Usage: Gets center in local space
	Arguments: ---
	Output: center
	*/
	vector3 GetCenterLocal(void);
	/*
	Usage: Gets minimum vector in local space
	Arguments: ---
	Output: min vector
	*/
	vector3 GetMinLocal(void);
	/*
	Usage: Gets maximum vector in local space
	Arguments: ---
	Output: max vector
	*/
	vector3 GetMaxLocal(void);
	/*
	Usage: Gets center in global space
	Arguments: ---
	Output: center
	*/
	vector3 GetCenterGlobal(void);
	/*
	Usage: Gets minimum vector in local space
	Arguments: ---
	Output: min vector
	*/
	vector3 GetMinGlobal(void);
	/*
	Usage: Gets max vector in global space
	Arguments: ---
	Output: max vector
	*/
	vector3 GetMaxGlobal(void);
	/*
	Usage: Gets the size of the model divided by 2
	Arguments: ---
	Output: halfwidth vector
	*/
	vector3 GetHalfWidth(void);
	/*
	Usage: Gets Model to World matrix
	Arguments: ---
	Output: model to world matrix
	*/
	matrix4 GetModelMatrix(void);
	/*
	Usage: Sets Model to World matrix
	Arguments: Model to World matrix
	Output: ---
	*/
	void SetModelMatrix(matrix4 a_m4ModelMatrix);

#pragma endregion
	
private:
	/*
	Usage: Deallocates member fields
	Arguments: ---
	Output: ---
	*/
	void Release(void);
	/*
	Usage: Allocates member fields
	Arguments: ---
	Output: ---
	*/
	void Init(void);
	/*
	USAGE: This will apply the Separation Axis Test
	ARGUMENTS: MyRigidBody* const a_pOther -> other rigid body to test against
	OUTPUT: 0 for colliding, all other first axis that succeeds test
	*/
	uint SAT(MyRigidBody* p_Other);
	/*
	USAGE: Calculate all 15 projection axes for the SAT
	ARGUMENTS: MyRigidBody* const a_pOther -> other rigid body to test against
	OUTPUT: a std::vector of vector3s representing the 15 projection axes
	*/
	std::vector<vector3> CalculateProjectionAxes(MyRigidBody* p_Other);
	/*
	USAGE: Project the rigidbody's oriented bounding box onto the vector3 axis
	ARGUMENTS: MyRigidBody* p_OBB -> rigid body to project, vector3 axis to project on to, 
	reference float min value of projection, refrence float max value of projection
	OUTPUT: will change the reference min and max to the min amd max values from projecting the 8 verticies onto the axis
	*/
	void ProjectOBB(MyRigidBody* p_OBB, const vector3& axis, float& min, float& max);
	/*
	USAGE: Calculate the 8 vertices of the Orientated Bounding Box
	ARGUMENTS: --
	OUTPUT: a std::vector of vector3s, the 8 vertices of the Orientated Bounding Box
	*/
	std::vector<vector3> GetOBBVertices();
	/*
	USAGE: Gets the local axis index of separation from the axis SAT result
	ARGUMENTS: the axis of separation
	OUTPUT: an integer representation of which local axis is sperating two objects
	x = 1
	y = 2
	z = 3
	*/
	uint GetAxisIndex(const vector3& axis);
	/*
	USAGE: Draws a plane along the axis of separation between this object and a_pOther
	ARGUMENTS: the uint axis index, see above, const a_pOther -> the other rigid body that this object is not colliding with
	OUTPUT: --
	*/
	void DrawSeperationPlane(uint axisIndex, MyRigidBody* const a_pOther);

};//class

} //namespace BTX

#endif //__MYRIGIDBODY_H_

/*
USAGE:
ARGUMENTS: ---
OUTPUT: ---
*/
