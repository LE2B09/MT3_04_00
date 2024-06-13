#include <Novice.h>
#include <imgui.h>
#include "Math/MathFunction.h"

//バネの構造体
struct Spring
{
	Vector3 anchor;				//アンカー。固定された端の位置
	float naturalLength;		//自然長さ
	float stiffness;			//剛性、バネ定数ｋ
	float dampingCoefficirent;	//減衰係数
};

//ボールの構造体
struct Ball
{
	Vector3 position;			//ボールの位置
	Vector3 velocity;			//ボールの速度
	Vector3 acceleration;		//ボールの加速度
	float mass;					//ボールの質量
	float radius;				//ボールの半径
	unsigned int color;			//ボールの色
};

static const int kWindowWidth = 1280;
static const int kWindowHeight = 720;

const char kWindowTitle[] = "提出用課題";

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int)
{

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	MathFunction Func;

	int prevMouseX = 0;
	int prevMouseY = 0;
	bool isDragging = false;

	// バネが動いているかどうかのフラグ
	bool springActive = false;

	//デルタタイム
	float deltaTime = 1.0f / 60.0f;

	Vector3 translate{};
	Vector3 rotate{};

	Vector3 cameraTranslate = { 0.0f, 1.9f, -6.49f };
	Vector3 cameraRotate = { 0.26f, 0.0f, 0.0f };

	Spring spring{};
	spring.anchor = { 0.0f, 0.0f, 0.0 };
	spring.naturalLength = 1.0f;
	spring.stiffness = 100.0f;
	spring.dampingCoefficirent = 2.0f;

	Ball ball{};
	ball.position = { 1.2f, 0.0f, 0.0f };
	ball.mass = 2.0f;
	ball.radius = 0.05f;
	ball.color = BLUE;

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0)
	{
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		// マウス入力を取得
		POINT mousePosition;
		GetCursorPos(&mousePosition);

		///
		/// ↓更新処理ここから
		///

		// マウスドラッグによる回転制御
		if (Novice::IsPressMouse(1))
		{
			if (!isDragging)
			{
				isDragging = true;
				prevMouseX = mousePosition.x;
				prevMouseY = mousePosition.y;
			}
			else
			{
				int deltaX = mousePosition.x - prevMouseX;
				int deltaY = mousePosition.y - prevMouseY;
				rotate.y += deltaX * 0.01f; // 水平方向の回転
				rotate.x += deltaY * 0.01f; // 垂直方向の回転
				prevMouseX = mousePosition.x;
				prevMouseY = mousePosition.y;
			}
		}
		else
		{
			isDragging = false;
		}

		// マウスホイールで前後移動
		int wheel = Novice::GetWheel();
		if (wheel != 0)
		{
			cameraTranslate.z += wheel * 0.01f; // ホイールの回転方向に応じて前後移動
		}

		// ImGuiの制御
		ImGui::Begin("Spring Control");
		if (ImGui::Button("Start Spring")) {
			springActive = true; // バネの動きを開始
		}
		if (ImGui::Button("Reset Spring")) {
			springActive = false; // バネの動きを停止
			ball.position = { 1.2f, 0.0f, 0.0f };
			ball.velocity = { 0.0f, 0.0f, 0.0f };
			ball.acceleration = { 0.0f, 0.0f, 0.0f };
		}
		ImGui::End();


		if (springActive)
		{
			Vector3 diff = ball.position - spring.anchor;

			float length = Func.Length(diff);
			if (length != 0.0f)
			{
				Vector3 direction = Func.Normalize(diff);
				Vector3 restPosition = spring.anchor + direction * spring.naturalLength;
				Vector3 displacement = length * (ball.position - restPosition);
				Vector3 restoringForce = -spring.stiffness * displacement;
				Vector3 dampingForce = -spring.dampingCoefficirent * ball.velocity;
				Vector3 force = restoringForce + dampingForce;
				ball.acceleration = force / ball.mass;
			}

			ball.velocity += ball.acceleration * deltaTime;
			ball.position += ball.velocity * deltaTime;

			/*	ball.position.x += ball.velocity.x * deltaTime;
				ball.position.y += ball.velocity.y * deltaTime;*/
		}

		//各種行列の計算
		Matrix4x4 worldMatrix = Func.MakeAffineMatrix({ 1.0f,1.0f,1.0f }, rotate, translate);
		Matrix4x4 cameraMatrix = Func.MakeAffineMatrix({ 1.0f,1.0f,1.0f }, cameraRotate, cameraTranslate);
		Matrix4x4 viewWorldMatrix = Func.Inverse(worldMatrix);
		Matrix4x4 viewCameraMatrix = Func.Inverse(cameraMatrix);
		// 透視投影行列を作成
		Matrix4x4 projectionMatrix = Func.MakePerspectiveFovMatrix(0.45f, float(kWindowWidth) / float(kWindowHeight), 0.1f, 100.0f);
		//ビュー座標変換行列を作成
		Matrix4x4 viewProjectionMatrix = Func.Multiply(viewWorldMatrix, Func.Multiply(viewCameraMatrix, projectionMatrix));
		//ViewportMatrixビューポート変換行列を作成
		Matrix4x4 viewportMatrix = Func.MakeViewportMatrix(0.0f, 0.0f, float(kWindowWidth), float(kWindowHeight), 0.0f, 1.0f);

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		// Gridを描画
		Func.DrawGrid(viewProjectionMatrix, viewportMatrix);

		// Ballを描画
		//Sphere sphere{ ball.position, ball.radius };
		Func.DrawSphere(Sphere{ .center = ball.position,.radius = ball.radius }, viewProjectionMatrix, viewportMatrix, ball.color);

		// Springを描画
		// バネのアンカーとボールの位置を結ぶ線を描画
		Vector3 screenAnchor = Func.Transform(spring.anchor, Func.Multiply(viewProjectionMatrix, viewportMatrix));
		Vector3 screenBallPosition = Func.Transform(ball.position, Func.Multiply(viewProjectionMatrix, viewportMatrix));
		Novice::DrawLine((int)screenAnchor.x, (int)screenAnchor.y, (int)screenBallPosition.x, (int)screenBallPosition.y, WHITE); // 線の色を指定

		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0)
		{
			break;
		}
	}

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}
