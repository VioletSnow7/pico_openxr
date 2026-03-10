/* Copyright (2021-2023) Bytedance Ltd. and/or its affiliates, All rights reserved. */
#include <dirent.h>
#include "pch.h"
#include "common.h"
#include "options.h"
#include "application.h"
#include "controller.h"
#include "hand.h"
#include <common/xr_linear.h>
#include "logger.h"
#include "gui.h"
#include "ray.h"
#include "text.h"
#include "player.h"
#include "utils.h"
#include "graphicsplugin.h"
#include "cube.h"
#include "cad_renderer.h"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "scene_understanding.h"
#include "communication/dataInterface.h"  // cadDataManager 主调用接口
// ======= application.cpp 业务流程与架构分析 =======
//
// 【使用的核心引擎与第三方库】：
// 1. OpenXR API: PICO 官方支持的 VR/AR 标准接口，负责处理设备追踪、每帧视图渲染、手柄状态（定义在 openxr_program.cpp，但本类使用了其数据类型如 XrPosef）。
// 2. GLM (OpenGL Mathematics)
// 3. ImGui: 立即渲染模式的 GUI 库，用于绘制空间中的控制面板 (Dashboard)，方便运行时的参数调节。
// 4. cadDataManager (FlatBuffer): 自定义的 CAD 模型解析库，读取 .fb 格式的模型描述文件，将其展开为可渲染的数据。
// 5. FreeType: 通过 text.h/text.cpp 引入，用来在 3D 空间里渲染清晰的文字提示（如 CAD 模型的状态显示、设备信息等）。
// 
// 【核心执行流程】：
// 1. 初始化阶段 (initialize):
//    - 获取并缓存 OpenXR 实例和会话句柄（m_instance, m_session）。
//    - 初始化各个子模块：手柄 (Controller)、射线 (Ray)、UI面板 (Gui)、文字 (Text)、基础图形渲染器 (CubeRender)。
//    - Cad加载流程：调用 cadDataManager::DataInterface 解析 .fb 文件，利用获取到的 renderInfo 数组给 mCadRenderer 完成 OpenGL 缓冲区的绑定。
//
// 2. 输入事件处理 (inputEvent):
//    - 每一帧都会由上层通过该接口传入当前左右手柄的按键状态（ApplicationEvent）。
//    - 在此处理 Dashboard 的开关（Menu键）、CAD 模型的抓取与移动（Grip 侧键）、CAD 模型的缩放与旋转（Thumbstick 摇杆）。
// 
// 3. 画面渲染循环 (renderFrame):
//    - 由 OpenXR 的渲染管线（GraphicsPlugin::RenderView）每帧调用两次（左眼一次，右眼一次）。
//    - 串联起所有可视物体的渲染：
//      a. layout() 更新 UI 矩阵
//      b. 视频播放器 (Player)、控制面板 (showDashboard)
//      c. 凝视射线 (renderEyeTracking)、手柄/手势模型 (mController->render, renderHandTracking)
//      d. ★ 渲染业务模型 (renderCadModel)：将带有状态跟踪的真实模型绘制到场景中。
// =============================================================


namespace AppConfig {

    //  CAD 模型
    struct {
        // assets 目录内的相对路径，例如 "FBData/运输车.fb"
        const char* assetPath   = "FBData/jipuche.fb";
        // cadDataManager 内部使用的模型标识（不含扩展名）
        const char* modelName   = "jipuche";

        // 初始位置（米）：x=左右  y=上下  z=前后（负值=前方）
        glm::vec3   position    = glm::vec3(0.0f, 0.0f, -2.0f);

        // 初始缩放：CAD 数据单位通常是毫米(mm)，XR 场景单位是米(m)
        // 1mm = 0.001m，所以默认缩放为 0.001
        // 如果模型太大/太小可在这里调整
        float       scale       = 0.001f;
        // 初始旋转（四元数，默认无旋转）
        glm::quat   rotation    = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    } CAD;
}

class Application : public IApplication {
public:
    Application(const std::shared_ptr<struct Options>& options, const std::shared_ptr<IGraphicsPlugin>& graphicsPlugin);
    virtual ~Application() override;
    virtual bool initialize(const XrInstance instance, const XrSession session, Extentions* extentions) override;
    virtual void setHapticCallback(void* arg, hapticCallback hapticCb) override;
    virtual void setControllerPose(int leftright, const XrPosef& pose) override;
    virtual void setControllerPower(int leftright, int power) override;
    virtual void setGazeLocation(XrSpaceLocation& gazeLocation, std::vector<XrView>& views, float ipd, XrResult result = XR_SUCCESS) override;
    virtual void setHandJointLocation(XrHandJointLocationEXT* location) override;
    virtual void inputEvent(int leftright, const ApplicationEvent& event) override;
    virtual void renderFrame(const XrPosef& pose, const glm::mat4& project, const glm::mat4& view, int32_t eye) override;
    // ★ 方案A：场景理解接口 ★
    virtual void setSceneUnderstanding(std::shared_ptr<SceneUnderstanding> su) override;
private:
    void layout();
    void showDashboard(const glm::mat4& project, const glm::mat4& view);
    void showDashboardController();
    void showDeviceInformation(const glm::mat4& project, const glm::mat4& view);
    void renderEyeTracking(const glm::mat4& project, const glm::mat4& view, int32_t eye);
    void renderHandTracking(const glm::mat4& project, const glm::mat4& view);
    void getAllVideoFiles(const std::string& path, std::vector<std::string>& files);
    void startPlayVideo(const std::string& file);
    void renderCadModel(const glm::mat4& project, const glm::mat4& view);
    void haptic(int leftright, float amplitude, float frequency, float duration/*seconds*/);
    // Calculate the angle between the vector v and the plane normal vector n
    float angleBetweenVectorAndPlane(const glm::vec3& vector, const glm::vec3& normal);


    hapticCallback mHapticCallback;
    void* mHapticCallbackArg;

private:
    std::shared_ptr<IGraphicsPlugin> mGraphicsPlugin;
    std::shared_ptr<Controller> mController;
    std::shared_ptr<Ray> mEyeTrackingRay;
    std::shared_ptr<Gui> mPanel;
    std::shared_ptr<Text> mTextRender;
    std::shared_ptr<Player> mPlayer;
    glm::mat4 mControllerModel;
    XrPosef mControllerPose[HAND_COUNT];
    std::shared_ptr<CubeRender> mCubeRender;
    std::shared_ptr<CadRenderer> mCadRenderer;

    //openxr
    XrInstance m_instance;          //Keep the same naming as openxr_program.cpp
    XrSession m_session;
    Extentions* m_extentions;
    XrSpaceLocation m_gazeLocation;
    std::vector<XrView> m_views;
    float mIpd;
    XrHandJointLocationEXT m_jointLocations[HAND_COUNT][XR_HAND_JOINT_COUNT_EXT];

    //app data
    std::string mDeviceModel;
    std::string mDeviceOS;

    bool mIsShowDashboard = false;  // 默认不显示 Dashboard（按 Menu 键可手动打开）

    std::vector<std::string> mAllVideoFiles;
    int32_t mCount = 0;

    const ApplicationEvent *mControllerEvent[HAND_COUNT];


    // ========== 方案B+C：地面和桌面碰撞 ==========
    float mTableHeight = -1.0f;       // 桌面高度（-1 表示未设置，按 A 键标记）
    bool mTableHeightSet = false;     // 桌面高度是否已设置
    bool mGravityEnabled = true;      // 是否启用重力/碰撞（松手后立方体会落到表面上）

    // ========== 方案A：场景理解模块 ==========
    std::shared_ptr<SceneUnderstanding> mSceneUnderstanding;

    // ========== CAD 模型交互状态 ==========
    bool mCadGrabbing = false;
    int mCadGrabHand = -1;
    glm::vec3 mCadGrabOffset;
    glm::quat mCadGrabRotOffset;
};

std::shared_ptr<IApplication> createApplication(const std::shared_ptr<struct Options>& options, const std::shared_ptr<IGraphicsPlugin>& graphicsPlugin) {
    return std::make_shared<Application>(options, graphicsPlugin);
}

Application::Application(const std::shared_ptr<struct Options>& options, const std::shared_ptr<IGraphicsPlugin>& graphicsPlugin) {
    mGraphicsPlugin = graphicsPlugin;
    mController = std::make_shared<Controller>();
    mEyeTrackingRay = std::make_shared<Ray>();
    mPanel = std::make_shared<Gui>("dashboard");
    mTextRender = std::make_shared<Text>();
    mPlayer = std::make_shared<Player>();
    mHapticCallback = nullptr;
    mCubeRender = std::make_shared<CubeRender>();
    mCadRenderer = std::make_shared<CadRenderer>();
}

Application::~Application() {
}

void Application::getAllVideoFiles(const std::string& path, std::vector<std::string>& allFiles) {
    DIR *dir = opendir(path.c_str());
    if (dir == nullptr) {
        errorf("opendir %s error %d", path.c_str(), errno);
        return;
    }
    struct dirent *file;
    while ((file = readdir(dir)) != nullptr) {
        if (strcmp(file->d_name, ".") == 0 || strcmp(file->d_name, "..") == 0) {
            continue;
        }
        if (file->d_type == DT_DIR) {
            std::string path_next = path + "/" + file->d_name;
            getAllVideoFiles(path_next, allFiles);
        } else {
            std::string fileFullName = path + "/" + file->d_name;
            std::string extension = fileFullName.substr(fileFullName.find_last_of('.') + 1);
            std::transform(extension.begin(), extension.end(), extension.begin(), [](char& c) {
                return std::tolower(c);
            });
            if (extension == "mp4" || extension == "mkv" || extension == "avi") {
                allFiles.push_back(fileFullName);
            }
            //infof("count:%d file:%s", mCount++, fileFullName.c_str());
        }
    }
}

bool Application::initialize(const XrInstance instance, const XrSession session, Extentions* extentions) {
    m_instance = instance;
    m_session = session;
    m_extentions = extentions;

    // get device model
    char buffer[64] = {0};
    __system_property_get("sys.pxr.product.name", buffer);
    mDeviceModel = buffer;

    //get OS version
    __system_property_get("ro.build.id", buffer);
    //__system_property_get("ro.system.build.id", buffer); // You can also call this function, the result is the same
    mDeviceOS = buffer;

    mController->initialize(mDeviceModel);
    mEyeTrackingRay->initialize();
    mPanel->initialize(600, 800);  //set resolution
    mTextRender->initialize();
    mCubeRender->initialize();

    // ========= ★ 初始化 CAD 渲染器 ★ =========
    // 调试模式：每一步打印日志，通过 logcat -s CadLoader 观察
    infof("[CadLoader] Step 0: mCadRenderer->initialize() ...");
    if (mCadRenderer->initialize()) {
        infof("[CadLoader] Step 0: initialize OK");

        // ---- Step 1: 从 assets 读取 .fb 文件 ----
        const char* assetPath = AppConfig::CAD.assetPath;
        infof("[CadLoader] Step 1: readFileFromAssets(\"%s\") ...", assetPath);
        auto fbData = readFileFromAssets(assetPath);
        infof("[CadLoader] Step 1: done, size = %zu bytes", fbData.size());

        if (fbData.empty()) {
            infof("[CadLoader] Step 1 FAILED: asset empty! "
                  "请确认文件在 app/src/main/assets/%s", assetPath);
        } else {
            // ---- Step 2: parseModelData ----
            const std::string modelFileName = AppConfig::CAD.modelName;
            infof("[CadLoader] Step 2: parseModelData(\"%s\", data, %zu) ...",
                  modelFileName.c_str(), fbData.size());
            cadDataManager::DataInterface::parseModelData(
                modelFileName,
                fbData.data(),
                fbData.size()
            );
            infof("[CadLoader] Step 2: parseModelData returned");

            // ---- Step 3: setActiveDocumentData ----
            infof("[CadLoader] Step 3: setActiveDocumentData(\"%s\") ...", modelFileName.c_str());
            cadDataManager::DataInterface::setActiveDocumentData(modelFileName);
            infof("[CadLoader] Step 3: done");

            // ---- Step 4: getRenderInfoMap (推荐接口，按 protoId 分组) ----
            infof("[CadLoader] Step 4: getRenderInfoMap() ...");
            auto renderInfoMap = cadDataManager::DataInterface::getRenderInfoMap();
            infof("[CadLoader] Step 4: got %zu proto groups", renderInfoMap.size());

            if (renderInfoMap.empty()) {
                // 也尝试一下 getRenderInfo()（旧接口）
                infof("[CadLoader] Step 4b: trying getRenderInfo() ...");
                auto ri = cadDataManager::DataInterface::getRenderInfo();
                infof("[CadLoader] Step 4b: getRenderInfo returned %zu items", ri.size());

                if (ri.empty()) {
                    infof("[CadLoader] Step 4 WARNING: both APIs returned empty! "
                          "文件格式可能不匹配，或者 parseModelData 未能识别该 .fb 文件");
                } else {
                    infof("[CadLoader] Step 5: loadFromRenderInfos(%zu) ...", ri.size());
                    mCadRenderer->loadFromRenderInfos(ri);
                    infof("[CadLoader] Step 5: DONE");
                }
            } else {
                // 将所有 proto 的 RenderInfo 打平合并
                std::vector<cadDataManager::RenderInfo> allRenderInfos;
                for (auto& kv : renderInfoMap) {
                    infof("[CadLoader] Step 4:   proto='%s', items=%zu",
                          kv.first.c_str(), kv.second.size());
                    allRenderInfos.insert(allRenderInfos.end(),
                                          kv.second.begin(), kv.second.end());
                }
                infof("[CadLoader] Step 4: total RenderInfo items = %zu", allRenderInfos.size());

                // ---- Step 5: loadFromRenderInfos ----
                infof("[CadLoader] Step 5: loadFromRenderInfos(%zu) ...", allRenderInfos.size());
                mCadRenderer->loadFromRenderInfos(allRenderInfos);
                infof("[CadLoader] Step 5: DONE — model loaded and ready");
            }
        }
    } else {
        infof("[CadLoader] Step 0 FAILED: initialize() returned false");
    }

    const XrGraphicsBindingOpenGLESAndroidKHR *binding = reinterpret_cast<const XrGraphicsBindingOpenGLESAndroidKHR*>(mGraphicsPlugin->GetGraphicsBinding());
    mPlayer->initialize(binding->display);

    getAllVideoFiles("/sdcard", mAllVideoFiles);

    //copyFile("/sdcard/Pictures/Screenshots/20230426-105301.jpg", "/sdcard/Pictures/2.jpg");
    //refreshMedia("/sdcard/Pictures/");

    return true;
}

void Application::setHapticCallback(void* arg, hapticCallback hapticCb) {
    mHapticCallbackArg = arg;
    mHapticCallback = hapticCb;
}

void Application::setControllerPower(int leftright, int power) {
    mController->setPowerValue(leftright, power);
}

void Application::setControllerPose(int leftright, const XrPosef& pose) {
    XrMatrix4x4f model{};
    XrVector3f scale{1.0f, 1.0f, 1.0f};
    XrMatrix4x4f_CreateTranslationRotationScale(&model, &pose.position, &pose.orientation, &scale);
    glm::mat4 m = glm::make_mat4((float*)&model);
    mController->setModel(leftright, m);
    mControllerPose[leftright] = pose;
}

void Application::setGazeLocation(XrSpaceLocation& gazeLocation, std::vector<XrView>& views, float ipd, XrResult result) {
    mIpd = ipd;
    memcpy(&m_gazeLocation, &gazeLocation, sizeof(gazeLocation));
    m_views = views;
}

void Application::setHandJointLocation(XrHandJointLocationEXT* location) {
    memcpy(&m_jointLocations, location, sizeof(m_jointLocations));
}

void Application::startPlayVideo(const std::string& file) {
    //mPlayer->stop();
    mPlayer->start(file);
}

// ★ 方案A：接收场景理解模块引用 ★
void Application::setSceneUnderstanding(std::shared_ptr<SceneUnderstanding> su) {
    mSceneUnderstanding = su;
    if (su) {
        infof("SceneUnderstanding: Module connected to Application");
    }
}

void Application::inputEvent(int leftright, const ApplicationEvent& event) {
    mControllerEvent[leftright] = &event;

    if (event.controllerEventBit & CONTROLLER_EVENT_BIT_click_menu) {
        if (event.click_menu == true) {
            mIsShowDashboard = !mIsShowDashboard;
        }
    }


    // A 键标记桌面高度
    if (event.controllerEventBit & CONTROLLER_EVENT_BIT_click_a) {
        if (event.click_a == true) {
            XrPosef& handPose = mControllerPose[leftright];
            mTableHeight = handPose.position.y;
            mTableHeightSet = true;
            haptic(leftright, 0.8f, 1.0f, 0.15f);
            infof("Table height marked at Y=%f by hand %d", mTableHeight, leftright);
        }
    }

    // B 键清除桌面标记 / 触发场景扫描
    if (event.controllerEventBit & CONTROLLER_EVENT_BIT_click_b) {
        if (event.click_b == true) {
            mTableHeightSet = false;
            mTableHeight = -1.0f;
            haptic(leftright, 0.3f, 1.0f, 0.1f);
            infof("Table height cleared");
        }
    }

    // X 键触发场景扫描（方案A）
    if (event.controllerEventBit & CONTROLLER_EVENT_BIT_click_x) {
        if (event.click_x == true && mSceneUnderstanding) {
            mSceneUnderstanding->requestSceneCapture();
            haptic(leftright, 0.8f, 1.0f, 0.2f);
            infof("Scene capture requested (Plan A)");
        }
    }

    // ========== CAD 模型交互：任意手 Grip 抓取拖动 ==========
    if (event.controllerEventBit & CONTROLLER_EVENT_BIT_click_squeeze) {
        if (event.click_squeeze == true && !mCadGrabbing) {
            XrPosef& handPose = mControllerPose[leftright];
            glm::vec3 handPos(handPose.position.x, handPose.position.y, handPose.position.z);
            float distance = glm::length(handPos - mCadRenderer->getPosition());

            if (distance < mCadRenderer->getBoundingRadius() + 0.5f) {
                mCadGrabbing = true;
                mCadGrabHand = leftright;
                glm::quat handRot(handPose.orientation.w, handPose.orientation.x,
                                  handPose.orientation.y, handPose.orientation.z);
                glm::quat handRotInv = glm::inverse(handRot);
                mCadGrabOffset = handRotInv * (mCadRenderer->getPosition() - handPos);
                mCadGrabRotOffset = handRotInv * mCadRenderer->getRotation();
                haptic(leftright, 0.5f, 1.0f, 0.1f);
                infof("CAD model grabbed by hand %d", leftright);
            }
        } else if (event.click_squeeze == false && mCadGrabbing && mCadGrabHand == leftright) {
            mCadGrabbing = false;
            mCadGrabHand = -1;
            haptic(leftright, 0.3f, 1.0f, 0.05f);
            infof("CAD model released at (%.2f, %.2f, %.2f)",
                mCadRenderer->getPosition().x, mCadRenderer->getPosition().y, mCadRenderer->getPosition().z);
        }
    }

    // ========== CAD 模型交互：右手摇杆缩放 ==========
    if (leftright == HAND_RIGHT && (event.controllerEventBit & CONTROLLER_EVENT_BIT_value_thumbstick)) {
        float scaleChange = event.thumbstick_y * 0.0005f; // 每帧缩放量
        float newScale = glm::clamp(mCadRenderer->getScale() + scaleChange, 0.0001f, 1.0f);
        mCadRenderer->setScale(newScale);
    }

    // ========== CAD 模型交互：左手摇杆旋转 ==========
    if (leftright == HAND_LEFT && (event.controllerEventBit & CONTROLLER_EVENT_BIT_value_thumbstick)) {
        float rotSpeed = 0.03f;
        if (std::abs(event.thumbstick_x) > 0.1f) {
            glm::quat rotY = glm::angleAxis(event.thumbstick_x * rotSpeed, glm::vec3(0.0f, 1.0f, 0.0f));
            mCadRenderer->setRotation(rotY * mCadRenderer->getRotation());
        }
        if (std::abs(event.thumbstick_y) > 0.1f) {
            glm::quat rotX = glm::angleAxis(-event.thumbstick_y * rotSpeed, glm::vec3(1.0f, 0.0f, 0.0f));
            mCadRenderer->setRotation(rotX * mCadRenderer->getRotation());
        }
    }

    if (leftright == HAND_LEFT) {
        return;
    }
    if (event.controllerEventBit & CONTROLLER_EVENT_BIT_click_trigger) {
        mPanel->triggerEvent(event.click_trigger);
    }

}

void Application::layout() {
    glm::mat4 model = glm::mat4(1.0f);
    float scale = 1.0f;

    float width, height;
    mPanel->getWidthHeight(width, height);
    scale = 0.7;
    model = glm::translate(model, glm::vec3(-0.0f, -0.3f, -1.0f));
    model = glm::rotate(model, glm::radians(10.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    model = glm::scale(model, glm::vec3(scale * (width / height), scale, 1.0f));
    mPanel->setModel(model);

    model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(1.0f, -0.0f, -1.5f));
    model = glm::rotate(model, glm::radians(-20.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    model = glm::scale(model, glm::vec3(scale*2, scale, 1.0f));
    mPlayer->setModel(model);
}

void Application::haptic(int leftright, float amplitude, float frequency/*not used now*/, float duration/*seconds*/) {
    if (mHapticCallback) {
        mHapticCallback(mHapticCallbackArg, leftright, amplitude, frequency, duration);
    }
}

void Application::showDashboardController() {
#define HAND_BIT_LEFT HAND_LEFT+1
#define HAND_BIT_RIGHT HAND_RIGHT+1
#define SHOW_CONTROLLER_ROW_float(x)    ImGui::TableNextRow();\
                                        ImGui::TableNextColumn();\
                                        ImGui::Text("%s", MEMBER_NAME(ApplicationEvent, x));\
                                        ImGui::TableNextColumn();\
                                        ImGui::Text("%f", mControllerEvent[HAND_LEFT]->x);\
                                        ImGui::TableNextColumn();\
                                        ImGui::Text("%f", mControllerEvent[HAND_RIGHT]->x);

#define SHOW_CONTROLLER_ROW_bool(hand, x)   ImGui::TableNextRow();\
                                            ImGui::TableNextColumn();\
                                            ImGui::Text("%s", MEMBER_NAME(ApplicationEvent, x));\
                                            ImGui::TableNextColumn();\
                                            if (hand & HAND_BIT_LEFT && mControllerEvent[HAND_LEFT]->x) {\
                                                ImGui::Text("true");\
                                            }\
                                            ImGui::TableNextColumn();\
                                            if (hand & HAND_BIT_RIGHT && mControllerEvent[HAND_RIGHT]->x) {\
                                                ImGui::Text("true");\
                                            }  

    //controller event
    if (ImGui::CollapsingHeader("controller")) {
        const float TEXT_BASE_WIDTH = ImGui::CalcTextSize("A").x;
        const float TEXT_BASE_HEIGHT = ImGui::GetTextLineHeightWithSpacing();
        static ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY;
        if (ImGui::BeginTable("controller event", 3, flags, ImVec2(0.0f, TEXT_BASE_HEIGHT * 19), 0.0f)) {
            ImGui::TableSetupColumn("event name",        ImGuiTableColumnFlags_NoSort | ImGuiTableColumnFlags_WidthFixed,   0.0f);
            ImGui::TableSetupColumn("left controller",   ImGuiTableColumnFlags_NoSort | ImGuiTableColumnFlags_WidthFixed,   0.0f);
            ImGui::TableSetupColumn("right controller",  ImGuiTableColumnFlags_NoSort | ImGuiTableColumnFlags_WidthStretch, 0.0f);
            ImGui::TableSetupScrollFreeze(0, 1); // Make row always visible
            ImGui::TableHeadersRow();

            SHOW_CONTROLLER_ROW_float(trigger);
            SHOW_CONTROLLER_ROW_bool(HAND_BIT_LEFT + HAND_BIT_RIGHT, click_trigger);
            SHOW_CONTROLLER_ROW_bool(HAND_BIT_LEFT + HAND_BIT_RIGHT, touch_trigger);

            SHOW_CONTROLLER_ROW_float(thumbstick_x);
            SHOW_CONTROLLER_ROW_float(thumbstick_y);
            SHOW_CONTROLLER_ROW_bool(HAND_BIT_LEFT + HAND_BIT_RIGHT, click_thumbstck);
            SHOW_CONTROLLER_ROW_bool(HAND_BIT_LEFT + HAND_BIT_RIGHT, touch_thumbstick);

            SHOW_CONTROLLER_ROW_float(squeeze);
            SHOW_CONTROLLER_ROW_bool(HAND_BIT_LEFT + HAND_BIT_RIGHT, click_squeeze);

            SHOW_CONTROLLER_ROW_bool(HAND_BIT_RIGHT, click_a);
            SHOW_CONTROLLER_ROW_bool(HAND_BIT_RIGHT, click_b);
            SHOW_CONTROLLER_ROW_bool(HAND_BIT_LEFT, click_x);
            SHOW_CONTROLLER_ROW_bool(HAND_BIT_LEFT, click_y);
            
            SHOW_CONTROLLER_ROW_bool(HAND_BIT_RIGHT, touch_a);
            SHOW_CONTROLLER_ROW_bool(HAND_BIT_RIGHT, touch_b);
            SHOW_CONTROLLER_ROW_bool(HAND_BIT_LEFT, touch_x);
            SHOW_CONTROLLER_ROW_bool(HAND_BIT_LEFT, touch_y);

            SHOW_CONTROLLER_ROW_bool(HAND_BIT_LEFT, click_menu);

            ImGui::EndTable();
        }

        for (int i = HAND_LEFT; i < HAND_COUNT; i++) {
            if (mControllerEvent[i]->squeeze > 0) {
                haptic(i, mControllerEvent[i]->squeeze, 1.0f, 0.02);
            } else {
                haptic(i, 0.0f, 0, 0.0f);
            }
        }
    }
}

void Application::showDashboard(const glm::mat4& project, const glm::mat4& view) {

    // get refreshrate
    uint32_t count = 0;
    m_extentions->xrEnumerateDisplayRefreshRatesFB(m_session, 0, &count, nullptr);
    std::vector<float> refreshRate(count);
    m_extentions->xrEnumerateDisplayRefreshRatesFB(m_session, count, &count, refreshRate.data());
    float currentreFreshRate = 0;
    m_extentions->xrGetDisplayRefreshRateFB(m_session, &currentreFreshRate);
    int currentreFreshRateTmp = (int)currentreFreshRate;

    PlayModel playModel = mPlayer->getPlayStyle();

    const XrPosef& controllerPose = mControllerPose[1];
    glm::vec3 linePoint = glm::make_vec3((float*)&controllerPose.position);
    glm::vec3 lineDirection = mController->getRayDirection(1);
    mPanel->isIntersectWithLine(linePoint, lineDirection);

    mPanel->begin();
    if (ImGui::CollapsingHeader("information")) {
        ImGui::BulletText("device model: %s", mDeviceModel.c_str());
        ImGui::BulletText("device OS: %s", mDeviceOS.c_str());
    }
    
    //test controller
    showDashboardController();

    if (ImGui::CollapsingHeader("framerate")) {
        ImGui::RadioButton("72fps", (int*)&currentreFreshRateTmp, 72); 
        if (refreshRate.size() > 1 || currentreFreshRateTmp == 92) {
            ImGui::SameLine();
            ImGui::RadioButton("90fps", (int*)&currentreFreshRateTmp, 90); 
        }
    }

    if (ImGui::CollapsingHeader("sample options")) {
        if (ImGui::BeginTable("split", 2)) {
            ImGui::TableNextColumn(); ImGui::Checkbox("XR_FB_passthrough", &m_extentions->activePassthrough);
            if (m_extentions->isSupportEyeTracking) {
                ImGui::TableNextColumn(); ImGui::Checkbox("Eye Tracking", &m_extentions->activeEyeTracking);
            }
            ImGui::EndTable();
        }
    }

    int32_t selectFileIndex = -1;
    if (ImGui::CollapsingHeader("video player")) {
        ImGui::SeparatorText("play model");
        ImGui::RadioButton("2D",         (int*)&playModel, (int)playModel_2D); ImGui::SameLine();
        ImGui::RadioButton("2D-180",     (int*)&playModel, (int)playModel_2D_180); ImGui::SameLine();
        ImGui::RadioButton("2D-360",     (int*)&playModel, (int)playModel_2D_360); ImGui::SameLine();
        ImGui::RadioButton("3D-SBS",     (int*)&playModel, (int)playModel_3D_SBS); ImGui::SameLine();
        ImGui::RadioButton("3D-SBS-360", (int*)&playModel, (int)playModel_3D_SBS_360); ImGui::SameLine();
        ImGui::RadioButton("3D-OU",      (int*)&playModel, (int)playModel_3D_OU); ImGui::SameLine();
        ImGui::RadioButton("3D-OU-360",  (int*)&playModel, (int)playModel_3D_OU_360);

        if (ImGui::CollapsingHeader("select media file")) {
            const float TEXT_BASE_WIDTH = ImGui::CalcTextSize("A").x;
            const float TEXT_BASE_HEIGHT = ImGui::GetTextLineHeightWithSpacing();
            static ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY;
            if (ImGui::BeginTable("meida files", 2, flags, ImVec2(0.0f, TEXT_BASE_HEIGHT * 11), 0.0f)) {
                ImGui::TableSetupColumn("ID",   ImGuiTableColumnFlags_NoSort     | ImGuiTableColumnFlags_WidthFixed,   0.0f);
                ImGui::TableSetupColumn("name", ImGuiTableColumnFlags_NoSort     | ImGuiTableColumnFlags_WidthStretch, 0.0f);
                ImGui::TableSetupScrollFreeze(0, 1); // Make row always visible
                ImGui::TableHeadersRow();
                for (int32_t i = 0; i < mAllVideoFiles.size(); i++) {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    if (ImGui::Selectable(Fmt("%02d", i).c_str(), true, ImGuiSelectableFlags_SpanAllColumns)) {
                        selectFileIndex = i;
                    }
                    ImGui::TableNextColumn();
                    ImGui::Text("%s", mAllVideoFiles[i].c_str());
                }
                ImGui::EndTable();
            }
        }
    }
    if (selectFileIndex != -1) {
        infof("item:%d, exchange video file %s", selectFileIndex, mAllVideoFiles[selectFileIndex].c_str());
        startPlayVideo(mAllVideoFiles[selectFileIndex]);
    }

    ImGui::Text("This is some useful text.");

    mPanel->end();
    mPanel->render(project, view);

    mPlayer->setPlayStyle(playModel);

    if (currentreFreshRateTmp != (int)currentreFreshRate) {  //changed
        m_extentions->xrRequestDisplayRefreshRateFB(m_session, (float)currentreFreshRateTmp);
    }
}

/**
 * 显示设备信息文字
 * 在 3D 空间中渲染一行白色文字，显示设备型号和操作系统版本
 * 位置：用户右下方前方 1 米处，略微向左旋转 30°
 */
void Application::showDeviceInformation(const glm::mat4& project, const glm::mat4& view) {
    // 格式化设备信息文本（宽字符，支持中文）
    wchar_t text[1024] = {0};
    swprintf(text, 1024, L"model: %s, OS: %s", mDeviceModel.c_str(), mDeviceOS.c_str());

    // 构建模型矩阵：定义文字在 3D 空间中的位置、旋转和缩放
    glm::mat4 model = glm::mat4(1.0f);                                      // 单位矩阵
    model = glm::translate(model, glm::vec3(0.5f, -0.6f, -1.0f));           // 平移到右下方前方 1 米处
    model = glm::rotate(model, glm::radians(-30.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // 绕 Y 轴向左旋转 30°
    model = glm::scale(model, glm::vec3(0.5, 0.5, 1.0f));                   // 缩放到 50% 大小
    // 使用 FreeType 文字渲染器绘制白色文字
    mTextRender->render(project, view, model, text, wcslen(text), glm::vec3(1.0, 1.0, 1.0));
}

/**
 * 计算向量与平面之间的夹角
 * @param vector 方向向量（如眼动注视方向）
 * @param normal 平面的法向量
 * @return 向量与平面的夹角（弧度），用于眼动追踪坐标映射
 */
float Application::angleBetweenVectorAndPlane(const glm::vec3& vector, const glm::vec3& normal) {
    float dotProduct = glm::dot(vector, normal);      // 向量点积
    float lengthVector = glm::length(vector);          // 向量长度
    float lengthNormal = glm::length(normal);          // 法向量长度
    if (lengthNormal != 1.0f) {
        lengthNormal = 1.0f;  // 归一化处理
    }
    float cosAngle = dotProduct / (lengthVector * lengthNormal); // 计算余弦值
    float angleRadians = std::acos(cosAngle);                    // 反余弦得到弧度
    // 返回向量与平面的夹角（π/2 - 与法向量的夹角 = 与平面的夹角）
    return PI/2 - angleRadians;
}

/**
 * 渲染眼动追踪指示器
 * 功能：在注视方向渲染一条红色射线，并将注视方向映射为屏幕坐标 (x, y) 百分比
 * 条件：仅在设备支持眼动追踪且已启用时渲染
 * @param eye 当前渲染的眼睛（EYE_LEFT=0 或 EYE_RIGHT=1）
 */
void Application::renderEyeTracking(const glm::mat4& project, const glm::mat4& view, int32_t eye) {
    // 仅在设备支持眼动追踪且已激活时执行
    if (m_extentions->isSupportEyeTracking && m_extentions->activeEyeTracking) {
        if (m_views.size() == 0) {
            return; // 视图数据尚未就绪
        }

        // ---- 第1步：构建注视点的模型矩阵 ----
        // 将 OpenXR 的注视姿态（位置+旋转）转换为 glm 矩阵
        XrMatrix4x4f m{};
        XrVector3f scale{1.0f, 1.0f, 1.0f};
        XrMatrix4x4f_CreateTranslationRotationScale(&m, &m_gazeLocation.pose.position, &m_gazeLocation.pose.orientation, &scale);
        glm::mat4 model = glm::make_mat4((float*)&m);

        // 根据瞳距 (IPD) 调整左右眼的偏移量
        float halfIpd = mIpd / 2;
        if (eye == EYE_LEFT) {
            halfIpd = 0 - halfIpd; // 左眼向左偏移
        }
        model = glm::translate(model, glm::vec3(halfIpd, 0.0f, -0.2f)); // 向前偏移 0.2 米
        model = glm::scale(model, glm::vec3(1.0, 1.0, 1.0f));
        mEyeTrackingRay->setColor(1.0f, 0.0f, 0.0f); // 设置射线颜色为红色

        // ---- 第2步：将注视方向映射到屏幕坐标 ----
        // 获取注视方向的方向向量
        glm::vec3 direction = mEyeTrackingRay->getDirectionVector(model);

        // 获取当前眼睛的视图姿态矩阵
        XrMatrix4x4f m2{};
        XrMatrix4x4f_CreateTranslationRotationScale(&m2, &m_views[eye].pose.position, &m_views[eye].pose.orientation, &scale);
        glm::mat4 model2 = glm::make_mat4((float*)&m2);

        // 在视图空间前方 1 米处构建一个虚拟屏幕平面
        glm::vec3 pointO = glm::vec3(model2 * glm::vec4(0.0, 0.0, -1.0, 1.0f)); // 平面中心点
        glm::vec3 pointX = glm::vec3(model2 * glm::vec4(1.0, 0.0, -1.0, 1.0f)); // X方向参考点
        glm::vec3 pointY = glm::vec3(model2 * glm::vec4(0.0, 1.0, -1.0, 1.0f)); // Y方向参考点
        glm::vec3 normalYOZ = pointX - pointO; // 水平方向的法向量（用于计算X坐标）
        glm::vec3 normalXOZ = pointY - pointO; // 垂直方向的法向量（用于计算Y坐标）

        // 计算注视方向与水平/垂直平面的夹角
        float angleAndYOZ = angleBetweenVectorAndPlane(direction, normalYOZ);
        float angleAndXOZ = angleBetweenVectorAndPlane(direction, normalXOZ);

        // 根据夹角和 FOV 计算注视点在屏幕上的百分比坐标 (0.0 ~ 1.0)
        float x, y;
        if (angleAndYOZ < 0) {
            // 注视点在屏幕左半部分
            x = (1 - tanf(angleAndYOZ) / tanf(m_views[eye].fov.angleLeft)) * 0.5;
        } else {
            // 注视点在屏幕右半部分
            x = (1 + tanf(angleAndYOZ) / tanf(m_views[eye].fov.angleRight)) * 0.5;
        }
        if (angleAndXOZ) {
            // 注视点在屏幕上半部分
            y = (1 - tanf(angleAndXOZ) / tanf(m_views[eye].fov.angleUp)) * 0.5;
        } else {
            // 注视点在屏幕下半部分
            y = (1 + tanf(angleAndXOZ) / tanf(m_views[eye].fov.angleDown)) * 0.5;
        }

        // ---- 第3步：渲染注视射线 ----
        mEyeTrackingRay->render(project, view, model);

        // ---- 第4步：在射线旁显示注视坐标文字 ----
        wchar_t text[1024] = {0};
        swprintf(text, 1024, L"x:%0.2f, y:%0.2f", x, y);
        model = glm::translate(model, glm::vec3(-0.2f, 0.0f, -1.5f)); // 文字位置偏移
        model = glm::scale(model, glm::vec3(0.5, 0.5, 0.5f));         // 文字缩放
        mTextRender->render(project, view, model, text, wcslen(text), glm::vec3(1.0, 1.0, 1.0));
    }
}

/**
 * 渲染手部追踪可视化
 * 功能：在每个检测到的手部关节位置渲染一个小立方体（1cm），形成手部骨骼的视觉表示
 * 每只手有 XR_HAND_JOINT_COUNT_EXT (26) 个关节点
 * 仅在关节位置有效且正在被追踪时才渲染对应的立方体
 */
void Application::renderHandTracking(const glm::mat4& project, const glm::mat4& view) {
    std::vector<CubeRender::Cube> cubes;
    // 遍历左右手
    for (auto hand = 0; hand < HAND_COUNT; hand++) {
        // 遍历每只手的 26 个关节（手腕、掌骨、指骨、指尖等）
        for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; i++) {
            XrHandJointLocationEXT& jointLocation = m_jointLocations[hand][i];
            // 检查关节位置是否有效且正在被追踪
            if (jointLocation.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT && jointLocation.locationFlags & XR_SPACE_LOCATION_POSITION_TRACKED_BIT) {

                // 将 OpenXR 关节姿态转换为 glm 模型矩阵
                XrMatrix4x4f m{};
                XrVector3f scale{1.0f, 1.0f, 1.0f};
                XrMatrix4x4f_CreateTranslationRotationScale(&m, &jointLocation.pose.position, &jointLocation.pose.orientation, &scale);
                glm::mat4 model = glm::make_mat4((float*)&m);

                // 创建一个 1cm 大小的立方体，放置在关节位置
                CubeRender::Cube cube;
                cube.model = model;
                cube.scale = 0.01f;  // 1厘米
                cubes.push_back(cube);
            }
        }
    }
    // 一次性批量渲染所有关节立方体
    mCubeRender->render(project, view, cubes);
}


/**
 * ★ 渲染 CAD 模型（含抓取位置跟踪） ★
 */
void Application::renderCadModel(const glm::mat4& project, const glm::mat4& view) {
    if (!mCadRenderer || !mCadRenderer->isLoaded()) return;

    // 如果正在抓取，更新 CAD 模型位置跟随左手柄
    if (mCadGrabbing && mCadGrabHand >= 0) {
        XrPosef& handPose = mControllerPose[mCadGrabHand];
        glm::vec3 handPos(handPose.position.x, handPose.position.y, handPose.position.z);
        glm::quat handRot(handPose.orientation.w, handPose.orientation.x,
                          handPose.orientation.y, handPose.orientation.z);
        mCadRenderer->setPosition(handPos + handRot * mCadGrabOffset);
        mCadRenderer->setRotation(handRot * mCadGrabRotOffset);
    }

    // 渲染 CAD 模型
    mCadRenderer->render(project, view);

    // 显示 CAD 模型状态提示
    wchar_t cadText[256] = {0};
    if (mCadGrabbing) {
        swprintf(cadText, 256, L"CAD: Grabbing | Scale: %.4f", mCadRenderer->getScale());
    } else {
        swprintf(cadText, 256, L"CAD: Grip=grab(any hand) L-Stick=rot R-Stick=scale | S:%.4f",
                 mCadRenderer->getScale());
    }
    glm::mat4 textModel = glm::mat4(1.0f);
    textModel = glm::translate(textModel, glm::vec3(-0.5f, -0.55f, -1.0f));
    textModel = glm::scale(textModel, glm::vec3(0.35f, 0.35f, 1.0f));
    mTextRender->render(project, view, textModel, cadText, wcslen(cadText), glm::vec3(0.3, 0.7, 1.0));
}

/**
 * ★ 应用层主渲染函数 ★
 * 
 * 每帧调用两次（左眼一次、右眼一次），由 GraphicsPlugin::RenderView() 调用。
 * 按照固定顺序渲染所有可视元素：
 *
 * @param pose    当前眼睛的姿态（位置 + 旋转）
 * @param project 投影矩阵（透视投影，基于 FOV）
 * @param view    视图矩阵（从眼睛姿态的逆矩阵得出）
 * @param eye     当前渲染的眼睛索引（0=左眼, 1=右眼）
 *
 * 渲染顺序（从后到前）：
 * 1. layout()                → 计算所有 UI 元素的布局位置
 * 2. showDeviceInformation() → 渲染设备信息文字（右下方白色小字）
 * 3. Player::render()        → 渲染视频播放画面（如果正在播放）
 * 4. showDashboard()         → 渲染 ImGui 控制面板（如果 Dashboard 开启）
 * 5. renderEyeTracking()     → 渲染眼动追踪射线和坐标（如果眼动追踪已启用）
 * 6. Controller::render()    → 渲染左右手柄 3D 模型和激光射线
 * 7. renderHandTracking()    → 渲染手部追踪关节立方体（如果手部被检测到）
 * 8. renderPlaceableCube()   → 渲染可交互的 50cm 立方体（可抓取和放置）
 */
void Application::renderFrame(const XrPosef& pose, const glm::mat4& project, const glm::mat4& view, int32_t eye) {
    // 1. 计算所有可视元素的空间布局（Dashboard面板、视频播放器的位置和大小）
    layout();

    // 2. 渲染设备信息文字（始终显示，位于右下方）
    showDeviceInformation(project, view);

    // 3. 渲染视频播放器画面（位于右前方，没有播放视频时不可见）
    mPlayer->render(project, view, eye);

    // 4. 渲染 Dashboard 控制面板（ImGui，包含刷新率/透视/眼动/视频选择等选项）
    if (mIsShowDashboard) {
        showDashboard(project, view);
    }

    // 5. 渲染眼动追踪指示器（红色射线 + 坐标文字，仅在眼动追踪启用时可见）
    renderEyeTracking(project, view, eye);

    // 6. 渲染左右手柄 3D 模型（PICO4/Neo3 手柄模型 + 激光射线指针）
    mController->render(project, view);

    // 7. 渲染手部追踪可视化（每个关节一个1cm立方体，仅在裸手被追踪时可见）
    renderHandTracking(project, view);

    // 渲染 CAD 模型
    renderCadModel(project, view);
}

