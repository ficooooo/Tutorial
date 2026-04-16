#ifndef DL_ROBOTCONTEXT_H

#define DL_ROBOTCONTEXT_H

#include <memory>

#include <QDomDocument>
#include <QString>

#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <AIS_Trihedron.hxx>
#include <gp_Ax1.hxx>
#include <gp_Trsf.hxx>
#include <rl/math/Transform.h>

class QWidget;
class gp_Dir;
class gp_Pnt;

namespace rl
{
namespace mdl
{
class Kinematic;
class Model;
class NloptInverseKinematics;
}
}

#define DL_ROBOT_JOINT_COUNT 6

//统一管理机器人在 OCC 显示层与 RL 运动学层之间的状态同步。
class DL_RobotContext
{
public:
    explicit DL_RobotContext(const Handle(AIS_InteractiveContext)& theContext);
    ~DL_RobotContext();
//--------------------------------------------------------------------------------------------
private://两个库共用的数据
    //当前生效的各关节角，单位为弧度。
    double      m_listJoints_angles[DL_ROBOT_JOINT_COUNT];
    //复位时回到的初始关节角，单位为弧度。
    double      m_listJoints_angles0[DL_ROBOT_JOINT_COUNT];
    //从 XML 解析出的各关节转轴，用于 OCC 端局部旋转。
    gp_Ax1      m_listJoints[DL_ROBOT_JOINT_COUNT];
    //当前加载的 Top.xml 入口文件。
    QString     m_topXmlFileName;
    //当前实际提供给 RL 模型的 robot.xml 文件。
    QString     m_robotXmlFileName;
    //机器人资源所在目录，也是相对 href 的解析基准目录。
    QString     m_robotDirPath;
    //当前单 STEP 预览文件，供拆分流程直接复用。
    QString     m_previewStepFileName;
    //杆件名称，按 base 到 flange 的顺序保存。
    QString     m_rodNames[DL_ROBOT_JOINT_COUNT + 1];
    //杆件几何文件路径，优先来自 Top.xml，缺省时回退旧命名规则。
    QString     m_rodFileNames[DL_ROBOT_JOINT_COUNT + 1];
    //当前已加载的 RL robot.xml 文档，同时作为 OCC 参数提取来源。
    QDomDocument m_robotXmlDocument;
public:
    //批量设置当前关节角，并立即刷新 OCC/RL 两侧状态。
    void          setPositions(const double* theAngles);
    //设置复位姿态使用的关节角基准值。
    void          setPositions0(const double* theAngles);
    //返回当前关节角数组首地址。
    double*       getPositions();
    //返回只读关节角数组首地址。
    const double* getPositions() const;
    //按 1-based 索引读取单个关节角。
    double        getPosition(int theJointIndex) const;
//--------------------------------------------------------------------------------------------
private://OCC显示数据  实现参考 robotcontext.cxx
    //7 个杆件显示对象，通过 AddChild 形成装配层级。
    Handle(AIS_Shape) m_listRods[DL_ROBOT_JOINT_COUNT + 1];
    // 从世界原点到当前末端位置的辅助轨迹线。
    Handle(AIS_Shape) m_traceLine;
    Handle(AIS_Shape) m_ASTool;//焊枪显示模型
    // 末端坐标系显示对象，用于可视化 TCP/法兰朝向。
    Handle(AIS_Trihedron) m_endTrihedron;
    // 当前窗口的 AIS 上下文。
    Handle(AIS_InteractiveContext) m_context;
public:
    // 按初始姿态恢复当前机器人。
    void              resetRobot();
    // 将当前关节角同步到 OCC 显示层与 RL 正解模型。
    void              forwardRobot();
    // 返回当前机器人杆件数量。
    int               getRodShapeCount() const;
    // 按索引访问单个杆件显示对象。
    Handle(AIS_Shape) getRodShape(int theIndex) const;
//--------------------------------------------------------------------------------------------
private://rl计算模型  实现参考 main_mdl_inv_ok.cpp
    //std::shared_ptr 用法见 std_shared_ptr.pptx
    std::shared_ptr<rl::mdl::Model>                  m_rl_mdl_Model;
    std::shared_ptr<rl::mdl::Kinematic>              m_rl_mdl_KinematicModel;
    std::shared_ptr<rl::mdl::NloptInverseKinematics> m_rl_mdl_IKSolver;
public://正逆解输入及输出参数要确认是否从工具末端开始
    // 对目标位姿做逆解，输出各关节角。
    bool                ikSolve(const rl::math::Transform& theTransform,
                                double                     theAngles[],
                                bool                       isFromTcp = false);
    // 对给定关节角做正解，返回末端位姿。
    rl::math::Transform forwardSolve(const double theAngles[], bool isToTcp = false);
//--------------------------------------------------------------------------------------------
private://以上两个库对象初始化
    // 按当前文件状态完成 RL 模型、OCC 杆件和初始姿态的整体装载。
    void              loadAll();
    // 加载并装配 OCC 杆件，同时根据 robot.xml 建立关节轴与末端坐标系。
    void              loadAISShapes();
    // 读取 STEP 并包装为 AIS_Shape。
    Handle(AIS_Shape) loadStp(const char* stpFileName);
    // 读取 IGES 并包装为 AIS_Shape。
    Handle(AIS_Shape) loadIges(const char* igesFileName);
    // 基于 RL XML 创建运动学模型。
    void              loadMdlModel(const char* xmlFileName);
//--------------------------------------------------------------------------------------------
private://与焊枪有关的模型和数据
    // TCP 相对法兰的位姿，以及其逆变换。
    gp_Trsf m_tfTCP;
    gp_Trsf m_tfTCPInv;
public:
    // 加载工具几何模型，并附着到末端杆件。
    void loadTool(const char* modelFileName);
    // 设置 TCP 变换，并同步到已加载的工具显示对象。
    void setTcp(const gp_Trsf& theTransform);

public://两个库之间的核心数据(结构)转换
    // 将 OCC 变换转换为 RL 变换。
    rl::math::Transform trans(const gp_Trsf& theTransform) const;
    // 将 RL 变换转换为 OCC 变换。
    gp_Trsf             trans(const rl::math::Transform& theTransform) const;
//--------------------------------------------------------------------------------------------
public:
    // 兼容旧目录入口：从目录中的 robot.xml + ROD_1_x.stp 装载机器人。
    int  loadRobot(const QString& theDirPath, QWidget* theParent = nullptr);
    // 当前 Load 按钮的统一入口：xml 走 Top.xml，stp 走拆分前预览。
    int  loadRobotDynamic(QWidget* theParent = nullptr);
    // 解析 Top.xml，并转换生成当前运行所需的 robot.xml。
    int  loadRobotFromXml(const QString& theXmlFileName, QWidget* theParent = nullptr);
    // 预览单个总装 STEP，不进入运动学装载流程。
    bool previewStepFile(const QString& theFileName, QWidget* theParent = nullptr);
    // 增量转动指定关节。
    void moveJoint(int theIndex, int theForward = 1);
    // 输出当前姿态下的正逆解验证信息。
    void calcRobot();
    // 将总装 STEP 按顶层子件拆分为多个 stp 文件。
    bool splitStepFile(const QString& theFileName);
    // 当前拆分按钮入口：优先拆当前预览的总装 STEP，否则弹框选择。
    void disasRobot(QWidget* theParent = nullptr);
    // 当前是否已成功装载可运动的机器人模型。
    bool isLoaded() const;

    // 弹出参数输入对话框，并生成 Top.xml + robot.xml。
    static void writeRobotXml(QWidget* theParent = nullptr);


private:
    // 移除当前机器人相关显示对象，并重置内部状态。
    void                  clearRobotPresentation();
    // 更新原点到末端的辅助轨迹线。
    void                  updateTraceLine();
    // 从 m_robotXmlFileName 读入并缓存 RL XML 文档。
    bool                  loadRobotXml();
    // 从 RL XML 中读取指定 fixed.translation 分量。
    double                getFixedVal(const char* theId, const char* theAxis) const;
    // 从 RL XML 中读取指定 revolute.axis。
    gp_Dir                getJointDir(int theIndex) const;
    // 创建末端显示坐标系，并缓存到 m_endTrihedron。
    Handle(AIS_Trihedron) createEndTrihedron(const gp_Pnt& thePoint,
                                             const gp_Dir& theNormal,
                                             const gp_Dir& theXAxis);
    // 在运行环境中查找示意图资源。
    static QString        findSampleImage(const char* theFileName);

private:
    // 由 robot.xml 抽取得到的关键几何参数，单位为 mm，仅供 OCC 端装配显示使用。
    double m_paramR;
    double m_paramH;
    double m_paramL2;
    double m_paramS;
    double m_paramL3;
    double m_paramD;
    double m_paramW;
    bool   m_isPose1;

public:
    // 视图切换时刷新当前 AIS 上下文。
    void setContext(const Handle(AIS_InteractiveContext)& theContext) { m_context = theContext; }
};

#endif
