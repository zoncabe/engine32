#include "physics/math/e32_transform.h"


Transform Transform::inverse() const
{
	Transform inv;
	inv.rotation = rotation.transposed();
	inv.position = inv.rotation.transform(-position);
	return inv;
}


Vector3 Transform::mulVector(const Vector3 &v) const
{
	return rotation.transform(v) + position;
}


Vector3 Transform::mulVectorScaled(const Vector3 &scale, const Vector3 &v) const
{
	Vector3 scaled = { scale.x * v.x, scale.y * v.y, scale.z * v.z };
	return rotation.transform(scaled) + position;
}


Transform Transform::operator*(const Transform &u) const
{
	Transform out;
	out.rotation = rotation * u.rotation;
	out.position = rotation.transform(u.position) + position;
	return out;
}


Vector3 Transform::mulVectorTransposed(const Vector3 &v) const
{
	return rotation.transformTransposed(v - position);
}


Transform Transform::productTransposed(const Transform &u) const
{
	Transform out;
	Matrix3 rt = rotation.transposed();
	out.rotation = rt * u.rotation;
	out.position = rt.transform(u.position - position);
	return out;
}


HalfSpace Transform::mulHalfSpace(const HalfSpace &p) const
{
	Vector3 worldOrigin = mulVector(p.origin());
	Vector3 worldNormal = rotation.transform(p.normal);
	return { worldNormal, worldOrigin.dot(worldNormal) };
}


HalfSpace Transform::mulHalfSpaceScaled(const Vector3 &scale, const HalfSpace &p) const
{
	Vector3 worldOrigin = mulVectorScaled(scale, p.origin());
	Vector3 worldNormal = rotation.transform(p.normal);
	return { worldNormal, worldOrigin.dot(worldNormal) };
}


HalfSpace Transform::mulHalfSpaceTransposed(const HalfSpace &p) const
{
	Vector3 localOrigin = mulVectorTransposed(p.origin());
	Vector3 localNormal = rotation.transformTransposed(p.normal);
	return { localNormal, localOrigin.dot(localNormal) };
}
