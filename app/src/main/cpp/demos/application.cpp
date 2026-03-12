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
//    - 初始化各个子模块：手柄 (Controller)、射线 (Ray)、基础图形渲染器 (CubeRender)。
//    - Cad加载流程：调用 cadDataManager::DataInterface 解析 .fb 文件，利用获取到的 renderInfo 数组给 mCadRenderer 完成 OpenGL 缓冲区的绑定。
//
// 2. 输入事件处理 (inputEvent):
//    - 每一帧都会由上层通过该接口传入当前左右手柄的按键状态（ApplicationEvent）。
//    - CAD 模型的抓取与移动（Grip 侧键）、CAD 模型的缩放与旋转（Thumbstick 摇杆）。
// 
// 3. 画面渲染循环 (renderFrame):
//    - 由 OpenXR 的渲染管线（GraphicsPlugin::RenderView）每帧调用两次（左眼一次，右眼一次）。
//    - 串联起所有可视物体的渲染：
//      a. 手柄模型 (mController->render)
//      b. ★ 渲染业务模型 (renderCadModel)：将带有状态跟踪的真实模型绘制到场景中。
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
        // 0.0001f代表缩小1000倍
        float scale = 0.01f;
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
private:
    void showDashboard(const glm::mat4& project, const glm::mat4& view);
    void showDashboardController();
    void renderCadModel(const glm::mat4& project, const glm::mat4& view);
    void haptic(int leftright, float amplitude, float frequency, float duration/*seconds*/);

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

    bool mIsShowDashboard = true;  // 默认不显示 Dashboard（按 Menu 键可手动打开）

    std::vector<std::string> mAllVideoFiles;
    int32_t mCount = 0;

    const ApplicationEvent *mControllerEvent[HAND_COUNT];


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

    // ========= 初始化 CAD 渲染器 =========
    // 调试方式：每一步打印日志，通过 logcat -s CadLoader 观察
    infof("[CadLoader] Step 1: mCadRenderer->initialize() ...");
    if (mCadRenderer->initialize()) {
        infof("[CadLoader] Step 1: initialize OK");

        // ---- Step 2: 从 assets 读取 .fb 文件 ----
        const char* assetPath = AppConfig::CAD.assetPath;
        infof("[CadLoader] Step 2: readFileFromAssets(\"%s\") ...", assetPath);
        auto fbData = readFileFromAssets(assetPath);
        infof("[CadLoader] Step 2: done, size = %zu bytes", fbData.size());

        if (fbData.empty()) {
            infof("[CadLoader] Step 2 FAILED: asset empty! "
                  "请确认文件在 app/src/main/assets/%s", assetPath);
        } else {
            // ---- Step 3: parseModelData ----
            const std::string modelFileName = AppConfig::CAD.modelName;
            infof("[CadLoader] Step 3: parseModelData(\"%s\", data, %zu) ...",
                  modelFileName.c_str(), fbData.size());
            cadDataManager::DataInterface::parseModelData(
                modelFileName,
                fbData.data(),
                fbData.size()
            );
            infof("[CadLoader] Step 3: parseModelData returned");

            // ---- Step 4: setActiveDocumentData ----
            infof("[CadLoader] Step 4: setActiveDocumentData(\"%s\") ...", modelFileName.c_str());
            cadDataManager::DataInterface::setActiveDocumentData(modelFileName);
            infof("[CadLoader] Step 4 done");

            // ---- Step 5: 加载材质数据 ----
            auto matData = readFileFromAssets("JsonData/CockpitMaterial.json");
            if (!matData.empty()) {
                std::string tmpMatPath = "/sdcard/CockpitMaterial.json";
                FILE* fp = fopen(tmpMatPath.c_str(), "wb");
                if (fp) {
                    fwrite(matData.data(), 1, matData.size(), fp);
                    fclose(fp);
                    infof("[CadLoader] Step 5: loadMaterialData(\"%s\") ...", tmpMatPath.c_str());
                    cadDataManager::DataInterface::loadMaterialData(tmpMatPath);
                } else {
                    infof("[CadLoader] Step 5 Error: Failed to write %s", tmpMatPath.c_str());
                }
            } else {
                infof("[CadLoader] Step 5 Warning: Failed to find JsonData/CockpitMaterial.json in assets");
            }

            // ---- Step 6: getRenderInfoMap (按 protoId 分组) ----
            infof("[CadLoader] Step 6: getRenderInfoMap() ...");
            auto renderInfoMap = cadDataManager::DataInterface::getRenderInfoMap();
            infof("[CadLoader] Step 6: got %zu proto groups", renderInfoMap.size());

            if (renderInfoMap.empty()) {
                // getRenderInfo()（旧接口）
                infof("[CadLoader] Step 6b: trying getRenderInfo() ...");
                auto ri = cadDataManager::DataInterface::getRenderInfo();
                infof("[CadLoader] Step 6b: getRenderInfo returned %zu items", ri.size());

                if (ri.empty()) {
                    infof("[CadLoader] Step 6 WARNING: both APIs returned empty! "
                          "文件格式可能不匹配，或者 parseModelData 未能识别该 .fb 文件");
                } else {
                    infof("[CadLoader] Step 7: loadFromRenderInfos(%zu) ...", ri.size());
                    mCadRenderer->loadFromRenderInfos(ri);
                    infof("[CadLoader] Step 7: DONE");
                }
            } else {
                // 将所有 proto 的 RenderInfo 打平合并
                std::vector<cadDataManager::RenderInfo> allRenderInfos;
                for (auto& kv : renderInfoMap) {
                    infof("[CadLoader] Step 6:   proto='%s', items=%zu",
                          kv.first.c_str(), kv.second.size());
                    allRenderInfos.insert(allRenderInfos.end(),
                                          kv.second.begin(), kv.second.end());
                }
                infof("[CadLoader] Step 6: total RenderInfo items = %zu", allRenderInfos.size());

                // ---- Step 7: loadFromRenderInfos ----
                infof("[CadLoader] Step 7: loadFromRenderInfos(%zu) ...", allRenderInfos.size());
                mCadRenderer->loadFromRenderInfos(allRenderInfos);
                
                // 将 AppConfig 里的设置应用给模型
                mCadRenderer->setPosition(AppConfig::CAD.position);
                mCadRenderer->setScale(AppConfig::CAD.scale);
                mCadRenderer->setRotation(AppConfig::CAD.rotation);
                
                infof("[CadLoader] Step 7: DONE — model loaded and ready");
            }
        }
    } else {
        infof("[CadLoader] Step 1 FAILED: initialize() returned false");
    }

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

void Application::inputEvent(int leftright, const ApplicationEvent& event) {
    mControllerEvent[leftright] = &event;

    if (event.controllerEventBit & CONTROLLER_EVENT_BIT_click_menu) {
        if (event.click_menu == true) {
            mIsShowDashboard = !mIsShowDashboard;
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

    mPanel->end();
    mPanel->render(project, view);

    mPlayer->setPlayStyle(playModel);

    if (currentreFreshRateTmp != (int)currentreFreshRate) {  //changed
        m_extentions->xrRequestDisplayRefreshRateFB(m_session, (float)currentreFreshRateTmp);
    }
}

/**
 * 渲染 CAD 模型（含抓取位置跟踪）
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
 * 应用层主渲染函数
 * 
 * 每帧调用两次（左眼一次、右眼一次），由 GraphicsPlugin::RenderView() 调用。
 * 按照固定顺序渲染所有可视元素：
 *
 * @param pose    当前眼睛的姿态（位置 + 旋转）
 * @param project 投影矩阵（透视投影，基于 FOV）
 * @param view    视图矩阵（从眼睛姿态的逆矩阵得出）
 * @param eye     当前渲染的眼睛索引（0=左眼, 1=右眼）
 *
 */
void Application::renderFrame(const XrPosef& pose, const glm::mat4& project, const glm::mat4& view, int32_t eye) {
    // 渲染 Dashboard 控制面板（ImGui，包含刷新率/透视/眼动/视频选择等选项）
    // 当前给关了
    if (mIsShowDashboard) {
        showDashboard(project, view);
    }

    // 渲染左右手柄 3D 模型（PICO4/Neo3 手柄模型 + 激光射线指针）
    mController->render(project, view);

    // 渲染 CAD 模型
    renderCadModel(project, view);
}

