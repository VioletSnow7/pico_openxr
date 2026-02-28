/* 
 * 场景理解模块 - Scene Understanding Module
 * 
 * 功能：使用 XR_FB_scene 系列扩展查询真实世界的平面（桌面、地面、墙面等）
 * 用途：让虚拟物体能够"放在"真实的桌子上、地面上
 * 
 * 依赖的 OpenXR 扩展：
 * - XR_FB_spatial_entity          (空间实体基础)
 * - XR_FB_spatial_entity_query    (查询空间锚点)
 * - XR_FB_spatial_entity_container(房间内所有实体)
 * - XR_FB_scene                   (语义标签、边界框)
 * - XR_FB_scene_capture           (触发房间扫描)
 */
#pragma once

#include <vector>
#include <string>
#include <openxr/openxr.h>
#include "common/gfxwrapper_opengl.h"
#include "logger.h"

// 检测到的平面信息
struct DetectedPlane {
    std::string label;       // 语义标签: "TABLE", "FLOOR", "WALL", "CEILING" 等
    XrSpace space;           // 平面的空间引用
    XrPosef pose;            // 平面在世界空间中的位姿
    float width;             // 平面宽度 (米)
    float height;            // 平面高度/深度 (米)
    float surfaceY;          // 平面顶部的 Y 坐标（用于碰撞检测）
    bool isHorizontal;       // 是否为水平面（桌面、地面）
    XrUuidEXT uuid;          // 唯一标识
};

class SceneUnderstanding {
public:
    SceneUnderstanding();
    ~SceneUnderstanding();

    // ====== 初始化 ======
    // 获取需要注册的 OpenXR 扩展列表
    static std::vector<const char*> GetRequiredExtensions();

    // 初始化函数指针（在 xrCreateInstance 之后调用）
    bool initializeFunctionPointers(XrInstance instance);

    // 初始化场景理解（在 session 创建后调用）
    bool initialize(XrSession session, XrSpace appSpace);

    // ====== 场景扫描 ======
    // 请求房间扫描（会打开系统的房间扫描界面）
    bool requestSceneCapture();

    // 查询已有的场景数据（不需要重新扫描）
    bool querySceneAnchors();

    // 处理异步事件回调
    void handleEvent(const XrEventDataBaseHeader* event);

    // ====== 获取数据 ======
    // 获取所有检测到的平面
    const std::vector<DetectedPlane>& getDetectedPlanes() const { return mPlanes; }

    // 获取最近的水平面高度（用于碰撞检测）
    // 返回在给定 (x, z) 位置下方最近的水平面的 Y 坐标
    // 如果没有找到，返回 0.0 (地面)
    float getNearestSurfaceY(float x, float y, float z) const;

    // 是否已经完成场景查询
    bool isSceneDataAvailable() const { return mSceneDataAvailable; }

    // 更新平面的空间位置（每帧调用）
    void updatePlaneLocations(XrTime predictedDisplayTime);

private:
    // 处理查询结果
    void processQueryResults();
    // 获取单个空间的语义标签
    std::string getSemanticLabel(XrSpace space);
    // 获取单个空间的 2D 边界框
    bool getBoundingBox2D(XrSpace space, float& width, float& height);

private:
    XrInstance mInstance = XR_NULL_HANDLE;
    XrSession mSession = XR_NULL_HANDLE;
    XrSpace mAppSpace = XR_NULL_HANDLE;

    // 检测到的平面列表
    std::vector<DetectedPlane> mPlanes;
    bool mSceneDataAvailable = false;

    // 异步请求 ID
    XrAsyncRequestIdFB mCaptureRequestId = 0;
    XrAsyncRequestIdFB mQueryRequestId = 0;
    bool mWaitingForCapture = false;
    bool mWaitingForQuery = false;

    // ====== XR_FB_scene 函数指针 ======
    PFN_xrGetSpaceSemanticLabelsFB pfnGetSpaceSemanticLabelsFB = nullptr;
    PFN_xrGetSpaceBoundingBox2DFB pfnGetSpaceBoundingBox2DFB = nullptr;
    PFN_xrGetSpaceBoundingBox3DFB pfnGetSpaceBoundingBox3DFB = nullptr;
    PFN_xrGetSpaceRoomLayoutFB pfnGetSpaceRoomLayoutFB = nullptr;
    PFN_xrGetSpaceBoundary2DFB pfnGetSpaceBoundary2DFB = nullptr;

    // ====== XR_FB_scene_capture 函数指针 ======
    PFN_xrRequestSceneCaptureFB pfnRequestSceneCaptureFB = nullptr;

    // ====== XR_FB_spatial_entity_query 函数指针 ======
    PFN_xrQuerySpacesFB pfnQuerySpacesFB = nullptr;
    PFN_xrRetrieveSpaceQueryResultsFB pfnRetrieveSpaceQueryResultsFB = nullptr;

    // ====== XR_FB_spatial_entity 函数指针 ======
    PFN_xrEnumerateSpaceSupportedComponentsFB pfnEnumerateSpaceSupportedComponentsFB = nullptr;
    PFN_xrSetSpaceComponentStatusFB pfnSetSpaceComponentStatusFB = nullptr;
    PFN_xrGetSpaceComponentStatusFB pfnGetSpaceComponentStatusFB = nullptr;

    // ====== XR_FB_spatial_entity_container 函数指针 ======
    PFN_xrGetSpaceContainerFB pfnGetSpaceContainerFB = nullptr;
};
