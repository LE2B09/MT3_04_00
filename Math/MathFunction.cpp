#include "MathFunction.h"
#include "Novice.h"

Vector3 MathFunction::Add(const Vector3& v1, const Vector3& v2)
{
	return Vector3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}

Vector3 MathFunction::Subtract(const Vector3& v1, const Vector3& v2)
{
	Vector3 result{};
	result.x = v1.x - v2.x;
	result.y = v1.y - v2.y;
	result.z = v1.z - v2.z;
	return result;
}

Vector3 MathFunction::Multiply(float scalar, const Vector3& v)
{
	Vector3 result{};
	result.x = scalar * v.x;
	result.y = scalar * v.y;
	result.z = scalar * v.z;
	return result;
}

float MathFunction::Dot(const Vector3& v1, const Vector3& v2)
{
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

float MathFunction::Length(const Vector3& v)
{
	return sqrtf(powf(v.x, 2) + powf(v.y, 2) + powf(v.z, 2));
}

Vector3 MathFunction::Normalize(const Vector3& v)
{
	float length = Length(v);
	Vector3 result{};
	if (length != 0.0) {
		result.x = v.x / length;
		result.y = v.y / length;
		result.z = v.z / length;
	}
	return result;
}

Vector3 MathFunction::Transform(const Vector3& vector, const Matrix4x4& matrix)
{
	Vector3 result{};
	result.x = vector.x * matrix.m[0][0] + vector.y * matrix.m[1][0] + vector.z * matrix.m[2][0] + 1.0f * matrix.m[3][0];
	result.y = vector.x * matrix.m[0][1] + vector.y * matrix.m[1][1] + vector.z * matrix.m[2][1] + 1.0f * matrix.m[3][1];
	result.z = vector.x * matrix.m[0][2] + vector.y * matrix.m[1][2] + vector.z * matrix.m[2][2] + 1.0f * matrix.m[3][2];
	float w = vector.x * matrix.m[0][3] + vector.y * matrix.m[1][3] + vector.z * matrix.m[2][3] + 1.0f * matrix.m[3][3];
	assert(w != 0.0f);
	result.x /= w;
	result.y /= w;
	result.z /= w;
	return result;
}

Vector3 MathFunction::Cross(const Vector3& v1, const Vector3& v2)
{
	Vector3 result{};
	result.x = v1.y * v2.z - v1.z * v2.y;
	result.y = v1.z * v2.x - v1.x * v2.z;
	result.z = v1.x * v2.y - v1.y * v2.x;
	return result;
}

Vector3 MathFunction::Project(const Vector3& v1, const Vector3& v2)
{
	return Multiply(Dot(v1, v2) / powf(Length(v2), 2), v2);
}

Vector3 MathFunction::ClosestPoint(const Vector3& point, const Segment& segment)
{
	// 線分の始点から終点へのベクトル
	Vector3 segmentVec = segment.diff;

	// 線分の始点からpointへのベクトル
	Vector3 pointToOrigin = Subtract(point, segment.origin);

	// 線分の始点からpointへのベクトルを、線分の方向ベクトルに投影し、線分上の点を求める
	float t = Dot(pointToOrigin, segmentVec) / Dot(segmentVec, segmentVec);

	// 線分上の最近接点
	Vector3 closestPointOnSegment = Add(segment.origin, Multiply(t, segmentVec));

	return closestPointOnSegment;
}

Vector3 MathFunction::Perpendicular(const Vector3& vector)
{
	if (vector.x != 0.0f || vector.z != 0.0f)
	{
		return { -vector.y,  vector.x ,0.0f };
	}
	return { 0.0f, -vector.z, vector.y }; // y軸のみの場合
}

Vector3 MathFunction::Lerp(const Vector3& v1, const Vector3& v2, float t)
{
	return Vector3(t * v1.x + (1.0f - t) * v2.x, t * v1.y + (1.0f - t) * v2.y, t * v1.z + (1.0f - t) * v2.z);
}

Matrix4x4 MathFunction::Add(const Matrix4x4& m1, const Matrix4x4& m2)
{
	Matrix4x4 result;
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			result.m[i][j] = m1.m[i][j] + m2.m[i][j];
		}
	}
	return result;
}

Matrix4x4 MathFunction::Subtract(const Matrix4x4& m1, const Matrix4x4& m2)
{
	Matrix4x4 result{};
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			result.m[i][j] = m1.m[i][j] - m2.m[i][j];
		}
	}
	return result;
}

Matrix4x4 MathFunction::Multiply(const Matrix4x4& m1, const Matrix4x4& m2)
{
	Matrix4x4 result{};
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			for (int k = 0; k < 4; k++)
			{
				result.m[i][j] += m1.m[i][k] * m2.m[k][j];
			}
		}
	}
	return result;
}

Matrix4x4 MathFunction::Inverse(const Matrix4x4& matrix)
{
	Matrix4x4 result{};

	float det = matrix.m[0][0] * (matrix.m[1][1] * matrix.m[2][2] * matrix.m[3][3] + matrix.m[1][2] * matrix.m[2][3] * matrix.m[3][1] + matrix.m[1][3] * matrix.m[2][1] * matrix.m[3][2] -
		matrix.m[1][3] * matrix.m[2][2] * matrix.m[3][1] - matrix.m[1][2] * matrix.m[2][1] * matrix.m[3][3] - matrix.m[1][1] * matrix.m[2][3] * matrix.m[3][2]) -
		matrix.m[0][1] * (matrix.m[1][0] * matrix.m[2][2] * matrix.m[3][3] + matrix.m[1][2] * matrix.m[2][3] * matrix.m[3][0] + matrix.m[1][3] * matrix.m[2][0] * matrix.m[3][2] -
			matrix.m[1][3] * matrix.m[2][2] * matrix.m[3][0] - matrix.m[1][2] * matrix.m[2][0] * matrix.m[3][3] - matrix.m[1][0] * matrix.m[2][3] * matrix.m[3][2]) +
		matrix.m[0][2] * (matrix.m[1][0] * matrix.m[2][1] * matrix.m[3][3] + matrix.m[1][1] * matrix.m[2][3] * matrix.m[3][0] + matrix.m[1][3] * matrix.m[2][0] * matrix.m[3][1] -
			matrix.m[1][3] * matrix.m[2][1] * matrix.m[3][0] - matrix.m[1][1] * matrix.m[2][0] * matrix.m[3][3] - matrix.m[1][0] * matrix.m[2][3] * matrix.m[3][1]) -
		matrix.m[0][3] * (matrix.m[1][0] * matrix.m[2][1] * matrix.m[3][2] + matrix.m[1][1] * matrix.m[2][2] * matrix.m[3][0] + matrix.m[1][2] * matrix.m[2][0] * matrix.m[3][1] -
			matrix.m[1][2] * matrix.m[2][1] * matrix.m[3][0] - matrix.m[1][1] * matrix.m[2][0] * matrix.m[3][2] - matrix.m[1][0] * matrix.m[2][2] * matrix.m[3][1]);

	result.m[0][0] = (matrix.m[1][1] * matrix.m[2][2] * matrix.m[3][3] + matrix.m[1][2] * matrix.m[2][3] * matrix.m[3][1] + matrix.m[1][3] * matrix.m[2][1] * matrix.m[3][2] -
		matrix.m[1][3] * matrix.m[2][2] * matrix.m[3][1] - matrix.m[1][2] * matrix.m[2][1] * matrix.m[3][3] - matrix.m[1][1] * matrix.m[2][3] * matrix.m[3][2]) /
		det;
	result.m[0][1] = (-matrix.m[0][1] * matrix.m[2][2] * matrix.m[3][3] - matrix.m[0][2] * matrix.m[2][3] * matrix.m[3][1] - matrix.m[0][3] * matrix.m[2][1] * matrix.m[3][2] +
		matrix.m[0][3] * matrix.m[2][2] * matrix.m[3][1] + matrix.m[0][2] * matrix.m[2][1] * matrix.m[3][3] + matrix.m[0][1] * matrix.m[2][3] * matrix.m[3][2]) /
		det;
	result.m[0][2] = (matrix.m[0][1] * matrix.m[1][2] * matrix.m[3][3] + matrix.m[0][2] * matrix.m[1][3] * matrix.m[3][1] + matrix.m[0][3] * matrix.m[1][1] * matrix.m[3][2] -
		matrix.m[0][3] * matrix.m[1][2] * matrix.m[3][1] - matrix.m[0][2] * matrix.m[1][1] * matrix.m[3][3] - matrix.m[0][1] * matrix.m[1][3] * matrix.m[3][2]) /
		det;
	result.m[0][3] = (-matrix.m[0][1] * matrix.m[1][2] * matrix.m[2][3] - matrix.m[0][2] * matrix.m[1][3] * matrix.m[2][1] - matrix.m[0][3] * matrix.m[1][1] * matrix.m[2][2] +
		matrix.m[0][3] * matrix.m[1][2] * matrix.m[2][1] + matrix.m[0][2] * matrix.m[1][1] * matrix.m[2][3] + matrix.m[0][1] * matrix.m[1][3] * matrix.m[2][2]) /
		det;

	result.m[1][0] = (-matrix.m[1][0] * matrix.m[2][2] * matrix.m[3][3] - matrix.m[1][2] * matrix.m[2][3] * matrix.m[3][0] - matrix.m[1][3] * matrix.m[2][0] * matrix.m[3][2] +
		matrix.m[1][3] * matrix.m[2][2] * matrix.m[3][0] + matrix.m[1][2] * matrix.m[2][0] * matrix.m[3][3] + matrix.m[1][0] * matrix.m[2][3] * matrix.m[3][2]) /
		det;
	result.m[1][1] = (matrix.m[0][0] * matrix.m[2][2] * matrix.m[3][3] + matrix.m[0][2] * matrix.m[2][3] * matrix.m[3][0] + matrix.m[0][3] * matrix.m[2][0] * matrix.m[3][2] -
		matrix.m[0][3] * matrix.m[2][2] * matrix.m[3][0] - matrix.m[0][2] * matrix.m[2][0] * matrix.m[3][3] - matrix.m[0][0] * matrix.m[2][3] * matrix.m[3][2]) /
		det;
	result.m[1][2] = (-matrix.m[0][0] * matrix.m[1][2] * matrix.m[3][3] - matrix.m[0][2] * matrix.m[1][3] * matrix.m[3][0] - matrix.m[0][3] * matrix.m[1][0] * matrix.m[3][2] +
		matrix.m[0][3] * matrix.m[1][2] * matrix.m[3][0] + matrix.m[0][2] * matrix.m[1][0] * matrix.m[3][3] + matrix.m[0][0] * matrix.m[1][3] * matrix.m[3][2]) /
		det;
	result.m[1][3] = (matrix.m[0][0] * matrix.m[1][2] * matrix.m[2][3] + matrix.m[0][2] * matrix.m[1][3] * matrix.m[2][0] + matrix.m[0][3] * matrix.m[1][0] * matrix.m[2][2] -
		matrix.m[0][3] * matrix.m[1][2] * matrix.m[2][0] - matrix.m[0][2] * matrix.m[1][0] * matrix.m[2][3] - matrix.m[0][0] * matrix.m[1][3] * matrix.m[2][2]) /
		det;

	result.m[2][0] = (matrix.m[1][0] * matrix.m[2][1] * matrix.m[3][3] + matrix.m[1][1] * matrix.m[2][3] * matrix.m[3][0] + matrix.m[1][3] * matrix.m[2][0] * matrix.m[3][1] -
		matrix.m[1][3] * matrix.m[2][1] * matrix.m[3][0] - matrix.m[1][1] * matrix.m[2][0] * matrix.m[3][3] - matrix.m[1][0] * matrix.m[2][3] * matrix.m[3][1]) /
		det;
	result.m[2][1] = (-matrix.m[0][0] * matrix.m[2][1] * matrix.m[3][3] - matrix.m[0][1] * matrix.m[2][3] * matrix.m[3][0] - matrix.m[0][3] * matrix.m[2][0] * matrix.m[3][1] +
		matrix.m[0][3] * matrix.m[2][1] * matrix.m[3][0] + matrix.m[0][1] * matrix.m[2][0] * matrix.m[3][3] + matrix.m[0][0] * matrix.m[2][3] * matrix.m[3][1]) /
		det;
	result.m[2][2] = (matrix.m[0][0] * matrix.m[1][1] * matrix.m[3][3] + matrix.m[0][1] * matrix.m[1][3] * matrix.m[3][0] + matrix.m[0][3] * matrix.m[1][0] * matrix.m[3][1] -
		matrix.m[0][3] * matrix.m[1][1] * matrix.m[3][0] - matrix.m[0][1] * matrix.m[1][0] * matrix.m[3][3] - matrix.m[0][0] * matrix.m[1][3] * matrix.m[3][1]) /
		det;
	result.m[2][3] = (-matrix.m[0][0] * matrix.m[1][1] * matrix.m[2][3] - matrix.m[0][1] * matrix.m[1][3] * matrix.m[2][0] - matrix.m[0][3] * matrix.m[1][0] * matrix.m[2][1] +
		matrix.m[0][3] * matrix.m[1][1] * matrix.m[2][0] + matrix.m[0][1] * matrix.m[1][0] * matrix.m[2][3] + matrix.m[0][0] * matrix.m[1][3] * matrix.m[2][1]) /
		det;

	result.m[3][0] = (-matrix.m[1][0] * matrix.m[2][1] * matrix.m[3][2] - matrix.m[1][1] * matrix.m[2][2] * matrix.m[3][0] - matrix.m[1][2] * matrix.m[2][0] * matrix.m[3][1] +
		matrix.m[1][2] * matrix.m[2][1] * matrix.m[3][0] + matrix.m[1][1] * matrix.m[2][0] * matrix.m[3][2] + matrix.m[1][0] * matrix.m[2][2] * matrix.m[3][1]) /
		det;
	result.m[3][1] = (matrix.m[0][0] * matrix.m[2][1] * matrix.m[3][2] + matrix.m[0][1] * matrix.m[2][2] * matrix.m[3][0] + matrix.m[0][2] * matrix.m[2][0] * matrix.m[3][1] -
		matrix.m[0][2] * matrix.m[2][1] * matrix.m[3][0] - matrix.m[0][1] * matrix.m[2][0] * matrix.m[3][2] - matrix.m[0][0] * matrix.m[2][2] * matrix.m[3][1]) /
		det;
	result.m[3][2] = (-matrix.m[0][0] * matrix.m[1][1] * matrix.m[3][2] - matrix.m[0][1] * matrix.m[1][2] * matrix.m[3][0] - matrix.m[0][2] * matrix.m[1][0] * matrix.m[3][1] +
		matrix.m[0][2] * matrix.m[1][1] * matrix.m[3][0] + matrix.m[0][1] * matrix.m[1][0] * matrix.m[3][2] + matrix.m[0][0] * matrix.m[1][2] * matrix.m[3][1]) /
		det;
	result.m[3][3] = (matrix.m[0][0] * matrix.m[1][1] * matrix.m[2][2] + matrix.m[0][1] * matrix.m[1][2] * matrix.m[2][0] + matrix.m[0][2] * matrix.m[1][0] * matrix.m[2][1] -
		matrix.m[0][2] * matrix.m[1][1] * matrix.m[2][0] - matrix.m[0][1] * matrix.m[1][0] * matrix.m[2][2] - matrix.m[0][0] * matrix.m[1][2] * matrix.m[2][1]) /
		det;

	return result;
}

Matrix4x4 MathFunction::Transpose(const Matrix4x4& m)
{
	Matrix4x4 result{};
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			result.m[i][j] = m.m[j][i];
		}
	}
	return result;
}

Matrix4x4 MathFunction::MakeIdentity()
{
	Matrix4x4 result{};
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			if (i == j)
			{
				result.m[i][j] = 1.0f;
			}
			else
			{
				result.m[i][j] = 0.0f;
			}
		}
	}
	return result;
}

Matrix4x4 MathFunction::MakeScaleMatrix(const Vector3& scale)
{
	Matrix4x4 result{};
	result.m[0][0] = scale.x;
	result.m[1][1] = scale.y;
	result.m[2][2] = scale.z;
	result.m[3][3] = 1.0f;
	return result;
}

Matrix4x4 MathFunction::MakeRotateXMatrix(float radian)
{
	Matrix4x4 result{};
	result.m[0][0] = 1.0f;
	result.m[1][1] = std::cos(radian);
	result.m[1][2] = std::sin(radian);
	result.m[2][1] = -std::sin(radian);
	result.m[2][2] = std::cos(radian);
	result.m[3][3] = 1.0f;
	return result;
}

Matrix4x4 MathFunction::MakeRotateYMatrix(float radian)
{
	Matrix4x4 result{};
	result.m[0][0] = std::cos(radian);
	result.m[0][2] = -std::sin(radian);
	result.m[1][1] = 1.0f;
	result.m[2][0] = std::sin(radian);
	result.m[2][2] = std::cos(radian);
	result.m[3][3] = 1.0f;
	return result;
}

Matrix4x4 MathFunction::MakeRotateZMatrix(float radian)
{
	Matrix4x4 result{};
	result.m[0][0] = std::cos(radian);
	result.m[0][1] = std::sin(radian);
	result.m[1][0] = -std::sin(radian);
	result.m[1][1] = std::cos(radian);
	result.m[2][2] = 1.0f;
	result.m[3][3] = 1.0f;
	return result;
}

Matrix4x4 MathFunction::MakeTranslateMatrix(const Vector3& translate)
{
	Matrix4x4 result{};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			if (i == j) {
				result.m[i][j] = 1.0f;
			}
		}
	}
	result.m[3][0] = translate.x;
	result.m[3][1] = translate.y;
	result.m[3][2] = translate.z;
	return result;
}

Matrix4x4 MathFunction::MakeAffineMatrix(const Vector3& scale, const Vector3& radian, const Vector3& translate)
{
	Matrix4x4 scaleMatrix = MakeScaleMatrix(scale);
	Matrix4x4 rotateXMatrix = MakeRotateXMatrix(radian.x);
	Matrix4x4 rotateYMatrix = MakeRotateYMatrix(radian.y);
	Matrix4x4 rotateZMatrix = MakeRotateZMatrix(radian.z);
	Matrix4x4 translateMatrix = MakeTranslateMatrix(translate);

	return Multiply(Multiply(Multiply(scaleMatrix, rotateXMatrix), rotateYMatrix), Multiply(rotateZMatrix, translateMatrix));

	//return Multiply(MakeScaleMatrix(scale), Multiply(Multiply(MakeRotateXMatrix(radian.x), Multiply(MakeRotateYMatrix(radian.y), MakeRotateZMatrix(radian.z))), MakeTranslateMatrix(translate)));
}

Matrix4x4 MathFunction::MakePerspectiveFovMatrix(float fovY, float aspectRatio, float nearClip, float farClip)
{
	Matrix4x4 result{};
	result.m[0][0] = 1.0f / aspectRatio * 1.0f / tanf(fovY / 2.0f);
	result.m[1][1] = 1.0f / tanf(fovY / 2.0f);
	result.m[2][2] = farClip / (farClip - nearClip);
	result.m[2][3] = 1.0f;
	result.m[3][2] = -farClip * nearClip / (farClip - nearClip);
	return result;
}

Matrix4x4 MathFunction::MakeOrthographicMatrix(float left, float top, float right, float bottom, float nearClip, float farClip)
{
	Matrix4x4 result{};
	result.m[0][0] = 2 / (right - left);
	result.m[1][1] = 2 / (top - bottom);
	result.m[2][2] = 1.0f / (farClip - nearClip);
	result.m[3][0] = (left + right) / (left - right);
	result.m[3][1] = (top + bottom) / (bottom - top);
	result.m[3][2] = nearClip / (nearClip - farClip);
	result.m[3][3] = 1.0f;
	return result;
}

Matrix4x4 MathFunction::MakeViewportMatrix(float left, float top, float width, float height, float minDepth, float maxDepth)
{
	Matrix4x4 result{};
	result.m[0][0] = width / 2.0f;
	result.m[1][1] = -height / 2.0f;
	result.m[2][2] = maxDepth - minDepth;
	result.m[3][0] = left + width / 2.0f;
	result.m[3][1] = top + height / 2.0f;
	result.m[3][2] = minDepth;
	result.m[3][3] = 1.0f;
	return result;
}

void MathFunction::DrawGrid(const Matrix4x4& ViewProjectionMatrix, const Matrix4x4& ViewportMatrix)
{
	//Grid用
	const float	kGridHalfWidth = 2.0f;										//Gridの半分の幅
	const uint32_t kSubdivision = 10;										//分割数
	const float kGridEvery = (kGridHalfWidth * 2.0f) / float(kSubdivision);	//1つ分の長さ

	//水平方向の線を描画
	for (uint32_t xIndex = 0; xIndex <= kSubdivision; xIndex++)
	{
		//上の情報を使ってワールド座標系上の始点と終点を求める
		//X軸上の座標
		float posX = -kGridHalfWidth + kGridEvery * xIndex;

		//始点と終点
		Vector3 start = { posX, 0.0f, -kGridHalfWidth };
		Vector3 end = { posX, 0.0f, kGridHalfWidth };
		//// ワールド座標系 -> スクリーン座標系まで変換をかける
		start = Transform(start, Multiply(ViewProjectionMatrix, ViewportMatrix));
		end = Transform(end, Multiply(ViewProjectionMatrix, ViewportMatrix));

		//左から右も同じように順々に引いていく
		for (uint32_t zIndex = 0; zIndex <= kSubdivision; zIndex++)
		{
			//奥から手前が左右に代わるだけ
			//上の情報を使ってワールド座標系上の始点と終点を求める
			//Z軸上の座標
			float posZ = -kGridHalfWidth + kGridEvery * zIndex;

			//始点と終点
			Vector3 startZ = { -kGridHalfWidth, 0.0f, posZ };
			Vector3 endZ = { kGridHalfWidth, 0.0f, posZ };
			//// ワールド座標系 -> スクリーン座標系まで変換をかける
			startZ = Transform(startZ, Multiply(ViewProjectionMatrix, ViewportMatrix));
			endZ = Transform(endZ, Multiply(ViewProjectionMatrix, ViewportMatrix));

			//変換した画像を使って表示。色は薄い灰色(0xAAAAAAFF)、原点は黒ぐらいがいいが、なんでもいい
			Novice::DrawLine((int)start.x, (int)start.y, (int)end.x, (int)end.y, 0x6F6F6FFF);
			Novice::DrawLine((int)startZ.x, (int)startZ.y, (int)endZ.x, (int)endZ.y, 0x6F6F6FFF);
		}
	}
}

void MathFunction::DrawSphere(const Sphere& sphere, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color)
{
	//球体用
	const uint32_t kSubdivision = 20;										//分割数
	const float kLatStep = (float)M_PI / kSubdivision;						//緯度のステップ
	const float kLonStep = 2.0f * (float)M_PI / kSubdivision;				//経度のステップ

	// 緯度のループ
	for (uint32_t latIndex = 0; latIndex < kSubdivision; ++latIndex)
	{
		float lat = -0.5f * (float)M_PI + latIndex * kLatStep;	//現在の緯度

		//次の緯度
		float nextLat = lat + kLatStep;

		//経度のループ
		for (uint32_t lonIndex = 0; lonIndex < kSubdivision; ++lonIndex)
		{
			//現在の経度
			float lon = lonIndex * kLonStep;

			//次の経度
			float nextLon = lon + kLonStep;

			// 球面座標の計算
			Vector3 pointA
			{
				sphere.center.x + sphere.radius * cos(lat) * cos(lon),
				sphere.center.y + sphere.radius * sin(lat),
				sphere.center.z + sphere.radius * cos(lat) * sin(lon)
			};

			Vector3 pointB
			{
				sphere.center.x + sphere.radius * cos(nextLat) * cos(lon),
				sphere.center.y + sphere.radius * sin(nextLat),
				sphere.center.z + sphere.radius * cos(nextLat) * sin(lon)
			};

			Vector3 pointC
			{
				sphere.center.x + sphere.radius * cos(lat) * cos(nextLon),
				sphere.center.y + sphere.radius * sin(lat),
				sphere.center.z + sphere.radius * cos(lat) * sin(nextLon)
			};

			// スクリーン座標に変換
			pointA = Transform(pointA, Multiply(viewProjectionMatrix, viewportMatrix));
			pointB = Transform(pointB, Multiply(viewProjectionMatrix, viewportMatrix));
			pointC = Transform(pointC, Multiply(viewProjectionMatrix, viewportMatrix));

			// 線分の描画
			Novice::DrawLine((int)pointA.x, (int)pointA.y, (int)pointB.x, (int)pointB.y, color);
			Novice::DrawLine((int)pointA.x, (int)pointA.y, (int)pointC.x, (int)pointC.y, color);
		}
	}
}

void MathFunction::DrawPlane(const Plane& plane, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color)
{
	Vector3 center = Multiply(plane.distance, plane.normal);
	Vector3 perpendiculars[4];
	perpendiculars[0] = Normalize(Perpendicular(plane.normal));
	perpendiculars[1] = { -perpendiculars[0].x,-perpendiculars[0].y,-perpendiculars[0].z };
	perpendiculars[2] = Cross(plane.normal, perpendiculars[0]);
	perpendiculars[3] = { -perpendiculars[2].x,-perpendiculars[2].y,-perpendiculars[2].z };

	// 平面の四隅を計算
	Vector3 points[4];
	for (int32_t index = 0; index < 4; index++)
	{
		Vector3 extend = Multiply(2.0f, perpendiculars[index]);
		Vector3 point = Add(center, extend);
		points[index] = Transform(Transform(point, viewProjectionMatrix), viewportMatrix);
	}

	Novice::DrawLine((int)points[0].x, (int)points[0].y, (int)points[2].x, (int)points[2].y, color);
	Novice::DrawLine((int)points[1].x, (int)points[1].y, (int)points[3].x, (int)points[3].y, color);
	Novice::DrawLine((int)points[2].x, (int)points[2].y, (int)points[1].x, (int)points[1].y, color);
	Novice::DrawLine((int)points[3].x, (int)points[3].y, (int)points[0].x, (int)points[0].y, color);
}

void MathFunction::DrawTriangle(const Triangle& triangle, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color)
{
	Vector3 screenVertices[3];
	for (int i = 0; i < 3; ++i)
	{
		screenVertices[i] = Transform(Transform(triangle.vertices[i], viewProjectionMatrix), viewportMatrix);
	}
	Novice::DrawTriangle((int)screenVertices[0].x, (int)screenVertices[0].y,
		(int)screenVertices[1].x, (int)screenVertices[1].y,
		(int)screenVertices[2].x, (int)screenVertices[2].y,
		color, kFillModeWireFrame);
}

void MathFunction::DrawAABB(const AABB& aabb, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color)
{
	Vector3 vertices[8];
	vertices[0] = { aabb.min.x, aabb.min.y, aabb.min.z };
	vertices[1] = { aabb.max.x, aabb.min.y, aabb.min.z };
	vertices[2] = { aabb.min.x, aabb.max.y, aabb.min.z };
	vertices[3] = { aabb.max.x, aabb.max.y, aabb.min.z };
	vertices[4] = { aabb.min.x, aabb.min.y, aabb.max.z };
	vertices[5] = { aabb.max.x, aabb.min.y, aabb.max.z };
	vertices[6] = { aabb.min.x, aabb.max.y, aabb.max.z };
	vertices[7] = { aabb.max.x, aabb.max.y, aabb.max.z };

	for (int i = 0; i < 8; ++i)
	{
		vertices[i] = Transform(vertices[i], Multiply(viewProjectionMatrix, viewportMatrix));
	}

	Novice::DrawLine((int)vertices[0].x, (int)vertices[0].y, (int)vertices[1].x, (int)vertices[1].y, color);
	Novice::DrawLine((int)vertices[0].x, (int)vertices[0].y, (int)vertices[2].x, (int)vertices[2].y, color);
	Novice::DrawLine((int)vertices[0].x, (int)vertices[0].y, (int)vertices[4].x, (int)vertices[4].y, color);
	Novice::DrawLine((int)vertices[1].x, (int)vertices[1].y, (int)vertices[3].x, (int)vertices[3].y, color);
	Novice::DrawLine((int)vertices[1].x, (int)vertices[1].y, (int)vertices[5].x, (int)vertices[5].y, color);
	Novice::DrawLine((int)vertices[2].x, (int)vertices[2].y, (int)vertices[3].x, (int)vertices[3].y, color);
	Novice::DrawLine((int)vertices[2].x, (int)vertices[2].y, (int)vertices[6].x, (int)vertices[6].y, color);
	Novice::DrawLine((int)vertices[3].x, (int)vertices[3].y, (int)vertices[7].x, (int)vertices[7].y, color);
	Novice::DrawLine((int)vertices[4].x, (int)vertices[4].y, (int)vertices[5].x, (int)vertices[5].y, color);
	Novice::DrawLine((int)vertices[4].x, (int)vertices[4].y, (int)vertices[6].x, (int)vertices[6].y, color);
	Novice::DrawLine((int)vertices[5].x, (int)vertices[5].y, (int)vertices[7].x, (int)vertices[7].y, color);
	Novice::DrawLine((int)vertices[6].x, (int)vertices[6].y, (int)vertices[7].x, (int)vertices[7].y, color);
}

void MathFunction::DrawBezier(const Vector3& controlPoint0, const Vector3& controlPoint1, const Vector3& controlPoint2, const Matrix4x4& viewProjection, const Matrix4x4& viewportMatrix, uint32_t color)
{
	const int kNumSegments = 100; // ベジエ曲線を描画するためのセグメント数

	for (int i = 0; i < kNumSegments; ++i)
	{
		float t1 = static_cast<float>(i) / kNumSegments;
		float t2 = static_cast<float>(i + 1) / kNumSegments;

		Vector3 point1 = Lerp(Lerp(controlPoint0, controlPoint1, t1), Lerp(controlPoint1, controlPoint2, t1), t1);
		Vector3 point2 = Lerp(Lerp(controlPoint0, controlPoint1, t2), Lerp(controlPoint1, controlPoint2, t2), t2);

		Vector3 screenPoint1 = Transform(point1, viewProjection);
		screenPoint1 = Transform(screenPoint1, viewportMatrix);

		Vector3 screenPoint2 = Transform(point2, viewProjection);
		screenPoint2 = Transform(screenPoint2, viewportMatrix);

		Novice::DrawLine((int)screenPoint1.x, (int)screenPoint1.y, (int)screenPoint2.x, (int)screenPoint2.y, color);
	}
}

void MathFunction::DrawControlPoint(const Vector3& controlPoint, const Matrix4x4& viewProjection, const Matrix4x4& viewportMatrix)
{
	Sphere sphere = { controlPoint, 0.01f };						// 0.01mの半径の球体
	DrawSphere(sphere, viewProjection, viewportMatrix, 0x000000);	// 黒色で描画
}

bool MathFunction::IsCollision(const Sphere& s1, const Sphere& s2)
{
	//2つの球の中心点間の距離を求める
	float distance = Length(Subtract(s2.center, s1.center));
	// 半径の合計よりも短ければ衝突
	return distance <= (s1.radius + s2.radius);
}

bool MathFunction::IsCollision(const Sphere& sphere, const Plane& plane)
{
	// 平面の法線ベクトルと球の中心点との距離
	float distance = Dot(plane.normal, sphere.center) - plane.distance;
	// その距離が球の半径以下なら衝突している
	return fabs(distance) <= sphere.radius;
}

bool MathFunction::IsCollision(const Segment& segment, const Plane& plane)
{
	//まず垂直判定を行うために、法線と線の内積を求める
	float dot = Dot(plane.normal, segment.diff);

	//垂直 = 平行であるので、衝突しているはずがない
	// 浮動小数点数の比較は通常、直接の等号判定は避ける
	const float epsilon = 1e-6f;
	if (fabs(dot) < epsilon)
	{
		return false;
	}

	//tを求める
	float t = (plane.distance - Dot(segment.origin, plane.normal)) / dot;

	//tの値と線の種類によって衝突しているかを判断する
	return t >= 0.0f && t <= 1.0f;
}

bool MathFunction::IsCollision(const Triangle& triangle, const Segment& segment)
{
	// 三角形の辺
	Vector3 edge1 = Subtract(triangle.vertices[1], triangle.vertices[0]);
	Vector3 edge2 = Subtract(triangle.vertices[2], triangle.vertices[0]);

	// 平面の法線ベクトルを計算
	Vector3 normal = Cross(edge1, edge2);
	normal = Normalize(normal);

	// 線分の方向ベクトル
	Vector3 dir = segment.diff;
	dir = Normalize(dir);

	// 平面と線分の始点のベクトル
	Vector3 diff = Subtract(triangle.vertices[0], segment.origin);

	// 線分が平面と平行かどうかをチェック
	float dotND = Dot(normal, dir);
	if (fabs(dotND) < 1e-6f)
	{
		return false; // 線分が平面と平行
	}

	// 線分の始点と平面の交点を計算
	float t = Dot(normal, diff) / dotND;

	if (t < 0.0f || t > Length(segment.diff))
	{
		return false; // 線分上に交点がない
	}

	Vector3 intersection = Add(segment.origin, Multiply(t, dir));

	// バリツチェックで三角形の内部に交点があるかを確認
	Vector3 c0 = Cross(Subtract(triangle.vertices[1], triangle.vertices[0]), Subtract(intersection, triangle.vertices[0]));
	Vector3 c1 = Cross(Subtract(triangle.vertices[2], triangle.vertices[1]), Subtract(intersection, triangle.vertices[1]));
	Vector3 c2 = Cross(Subtract(triangle.vertices[0], triangle.vertices[2]), Subtract(intersection, triangle.vertices[2]));

	if (Dot(c0, normal) >= 0.0f && Dot(c1, normal) >= 0.0f && Dot(c2, normal) >= 0.0f)
	{
		return true; // 衝突
	}

	return false; // 衝突なし
}

bool MathFunction::IsCollision(const AABB& aabb1, const AABB& aabb2)
{
	return (aabb1.min.x <= aabb2.max.x && aabb1.max.x >= aabb2.min.x) && //x軸
		(aabb1.min.y <= aabb2.max.y && aabb1.max.y >= aabb2.min.y) &&
		(aabb1.min.z <= aabb2.max.z && aabb1.max.z >= aabb2.min.z);
}

bool MathFunction::IsCollision(const AABB& aabb, const Sphere& sphere)
{
	//最近接点を求める
	Vector3 clossestPoint
	{
		std::clamp(sphere.center.x,aabb.min.x,aabb.max.x),
		std::clamp(sphere.center.y,aabb.min.y,aabb.max.y),
		std::clamp(sphere.center.z,aabb.min.z,aabb.max.z)
	};
	//最近接点と球の中途の距離を求める
	float distance = Length(Subtract(clossestPoint, sphere.center));
	//距離が半径よりも小さければ衝突
	return distance <= sphere.radius;
}

bool MathFunction::IsCollision(const AABB& aabb, const Segment& segment)
{
	float tNearX = (aabb.min.x - segment.origin.x) / segment.diff.x;
	float tFarX = (aabb.max.x - segment.origin.x) / segment.diff.x;
	if (tNearX > tFarX) std::swap(tNearX, tFarX);

	float tNearY = (aabb.min.y - segment.origin.y) / segment.diff.y;
	float tFarY = (aabb.max.y - segment.origin.y) / segment.diff.y;
	if (tNearY > tFarY) std::swap(tNearY, tFarY);

	float tNearZ = (aabb.min.z - segment.origin.z) / segment.diff.z;
	float tFarZ = (aabb.max.z - segment.origin.z) / segment.diff.z;
	if (tNearZ > tFarZ) std::swap(tNearZ, tFarZ);

	// 線分がAABBを貫通しているかどうかを判定
	float tmin = std::max(std::max(tNearX, tNearY), tNearZ);
	float tmax = std::min(std::min(tFarX, tFarY), tFarZ);

	// 衝突しているかどうかの判定
	if (tmin <= tmax && tmax >= 0.0f && tmin <= 1.0f) {
		return true;
	}
	return false;
}
