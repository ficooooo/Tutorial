#ifndef DL_ROBOTCONTEXT_H
#define DL_ROBOTCONTEXT_H

#include <memory>

#include <QDomDocument>
#include <QString>
#include <QStringList>

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

class DL_RobotContext
{
public:
    enum LoadMode
    {
        LoadMode_None = 0,
        LoadMode_AssemblyPreview,
        LoadMode_SplitPreview,
        LoadMode_RobotPackage
    };

    explicit DL_RobotContext(const Handle(AIS_InteractiveContext)& theContext);
    ~DL_RobotContext();
//--------------------------------------------------------------------------------------------
private://两个库共用的数据
    double      m_listJoints_angles[DL_ROBOT_JOINT_COUNT];
    double      m_listJoints_angles0[DL_ROBOT_JOINT_COUNT];
    gp_Ax1      m_listJoints[DL_ROBOT_JOINT_COUNT];
    QString     m_robotXmlFileName;
    QString     m_robotDirPath;
    QDomDocument m_robotXmlDocument;
public:
    void          setPositions(const double* theAngles);
    void          setPositions0(const double* theAngles);
    double*       getPositions();
    const double* getPositions() const;
    double        getPosition(int theJointIndex) const;
//--------------------------------------------------------------------------------------------
private://OCC显示数据  实现参考 robot.cxx
    Handle(AIS_Shape) m_listRods[DL_ROBOT_JOINT_COUNT + 1];
    Handle(AIS_Shape) m_traceLine;
    Handle(AIS_Shape) m_ASTool;//焊枪显示模型
    Handle(AIS_Trihedron) m_endTrihedron;
    Handle(AIS_InteractiveContext) m_context;
public:
    void              resetRobot();
    void              forwardRobot();
    int               getRodShapeCount() const;
    Handle(AIS_Shape) getRodShape(int theIndex) const;
//--------------------------------------------------------------------------------------------
private://rl计算模型  实现参考 main_mdl_inv_ok.cpp
    //std::shared_ptr 用法见 std_shared_ptr.pptx
    std::shared_ptr<rl::mdl::Model>                  m_rl_mdl_Model;
    std::shared_ptr<rl::mdl::Kinematic>              m_rl_mdl_KinematicModel;
    std::shared_ptr<rl::mdl::NloptInverseKinematics> m_rl_mdl_IKSolver;
public://正逆解输入及输出参数要确认是否从工具末端开始
    bool                ikSolve(const rl::math::Transform& theTransform,
                                double                     theAngles[],
                                bool                       isFromTcp = false);
    rl::math::Transform forwardSolve(const double theAngles[], bool isToTcp = false);
//--------------------------------------------------------------------------------------------
private://以上两个库对象初始化
    void              loadAll();
    void              loadAISShapes();
    Handle(AIS_Shape) loadStp(const char* stpFileName);
    Handle(AIS_Shape) loadIges(const char* igesFileName);
    void              loadMdlModel(const char* xmlFileName);
//--------------------------------------------------------------------------------------------
private://与焊枪有关的模型和数据
    gp_Trsf m_tfTCP;
    gp_Trsf m_tfTCPInv;
public:
    void loadTool(const char* modelFileName);
    void setTcp(const gp_Trsf& theTransform);

public://两个库之间的核心数据(结构)转换
    rl::math::Transform trans(const gp_Trsf& theTransform) const;
    gp_Trsf             trans(const rl::math::Transform& theTransform) const;
//--------------------------------------------------------------------------------------------
public:
    int  loadRobot(const QString& theDirPath, QWidget* theParent = nullptr);
    int  loadRobotDynamic(QWidget* theParent = nullptr);
    void moveJoint(int theIndex, int theForward = 1);
    void calcRobot();
    bool splitStepFile(const QString& theFileName);
    void disasRobot(QWidget* theParent = nullptr);
    bool isLoaded() const;
    bool canSplit() const;
    bool canWriteXml() const;
    bool canDrive() const;
    LoadMode loadMode() const;

    static void writeRobotXml(QWidget* theParent = nullptr);


private:
    void                  clearRobotPresentation();
    void                  updateTraceLine();
    bool                  loadRobotXml();
    double                getFixedVal(const char* theId, const char* theAxis) const;
    gp_Dir                getJointDir(int theIndex) const;
    Handle(AIS_Trihedron) createEndTrihedron(const gp_Pnt& thePoint,
                                             const gp_Dir& theNormal,
                                             const gp_Dir& theXAxis);
    static QString        findSampleImage(const char* theFileName);
    static QStringList    collectRodFiles(const QString& theDirectoryPath);
    int                   loadAssemblyPreview(const QString& theFileName, QWidget* theParent);
    int                   loadRobotPackage(const QString& theFileName, QWidget* theParent);
    bool                  parseMainXml(const QString& theFileName,
                                       QString&       theMdlFileName,
                                       QStringList&   theRodFileNames,
                                       QString&       theErrorMessage) const;

private:
    double m_paramR;
    double m_paramH;
    double m_paramL2;
    double m_paramS;
    double m_paramL3;
    double m_paramD;
    double m_paramW;
    bool   m_isPose1;
    LoadMode   m_loadMode;
    QString    m_robotMainFileName;
    QString    m_currentAssemblyFileName;
    QStringList m_rodFileNames;

public:
    void setContext(const Handle(AIS_InteractiveContext)& theContext) { m_context = theContext; }
};

#endif
