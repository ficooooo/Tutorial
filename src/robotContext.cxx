#include "DL_RobotContext.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <stdexcept>

#include <AIS_ListOfInteractive.hxx>
#include <AIS_Trihedron.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRep_Builder.hxx>
#include <Geom_Axis2Placement.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <IGESControl_Reader.hxx>
#include <Prs3d_DatumAspect.hxx>
#include <Quantity_Color.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Iterator.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <QCoreApplication>
#include <QDialog>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPixmap>
#include <QPushButton>
#include <QRadioButton>
#include <QStringList>
#include <QTextStream>
#include <QVBoxLayout>

#include <rl/math/Unit.h>
#include <rl/mdl/Joint.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/NloptInverseKinematics.h>
#include <rl/mdl/XmlFactory.h>

namespace
{
static const char* ROBOT_XML_TEMPLATE = R"(<?xml version="1.0" encoding="UTF-8"?><rlmdl xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="rlmdl.xsd"><model><manufacturer>Huashu</manufacturer><name>HSR-CR630-1750</name><world id="world"><rotation><x>0</x><y>0</y><z>0</z></rotation><translation><x>0</x><y>0</y><z>0</z></translation><g><x>0</x><y>0</y><z>9.86055</z></g></world><body id="body0"><ignore/><ignore idref="body1"/></body><frame id="frame0"/><frame id="frame1"/><body id="body1"><ignore idref="body0"/><ignore idref="body2"/></body><frame id="frame2"/><body id="body2"><ignore idref="body1"/><ignore idref="body3"/><ignore idref="body4"/></body><frame id="frame3"/><body id="body3"><ignore idref="body2"/><ignore idref="body4"/></body><frame id="frame4"/><body id="body4"><ignore idref="body2"/><ignore idref="body3"/><ignore idref="body5"/><ignore idref="body6"/></body><frame id="frame5"/><body id="body5"><ignore idref="body4"/><ignore idref="body6"/></body><frame id="frame6"/><body id="body6"><ignore idref="body4"/><ignore idref="body5"/></body><frame id="frame7"/><fixed id="fixed0"><frame><a idref="world"/><b idref="body0"/></frame><rotation><x>0</x><y>0</y><z>0</z></rotation><translation><x>0</x><y>0</y><z>0</z></translation></fixed><fixed id="fixed1"><frame><a idref="body0"/><b idref="frame0"/></frame><rotation><x>0</x><y>0</y><z>0</z></rotation><translation><x>0</x><y>0</y><z>0</z></translation></fixed><revolute id="joint0"><frame><a idref="frame0"/><b idref="frame1"/></frame><axis><x>0</x><y>0</y><z>1</z></axis><max>360</max><min>-360</min><speed>187</speed></revolute><fixed id="fixed2"><frame><a idref="frame1"/><b idref="body1"/></frame><rotation><x>0</x><y>0</y><z>0</z></rotation><translation><x>0</x><y>0</y><z>0.5015</z></translation></fixed><revolute id="joint1"><frame><a idref="body1"/><b idref="frame2"/></frame><axis><x>0</x><y>1</y><z>0</z></axis><max>360</max><min>-360</min><speed>160</speed></revolute><fixed id="fixed3"><frame><a idref="frame2"/><b idref="body2"/></frame><rotation><x>0</x><y>0</y><z>0</z></rotation><translation><x>0</x><y>0</y><z>0.950</z></translation></fixed><revolute id="joint2"><frame><a idref="body2"/><b idref="frame3"/></frame><axis><x>0</x><y>1</y><z>0</z></axis><max>360</max><min>-360</min><speed>180</speed></revolute><fixed id="fixed4"><frame><a idref="frame3"/><b idref="body3"/></frame><rotation><x>0</x><y>0</y><z>0</z></rotation><translation><x>0</x><y>0</y><z>0</z></translation></fixed><revolute id="joint3"><frame><a idref="body3"/><b idref="frame4"/></frame><axis><x>1</x><y>0</y><z>0</z></axis><max>360</max><min>-360</min><speed>260</speed></revolute><fixed id="fixed5"><frame><a idref="frame4"/><b idref="body4"/></frame><rotation><x>0</x><y>0</y><z>0</z></rotation><translation><x>0.615</x><y>0</y><z>0</z></translation></fixed><revolute id="joint4"><frame><a idref="body4"/><b idref="frame5"/></frame><axis><x>0</x><y>1</y><z>0</z></axis><max>360</max><min>-360</min><speed>230</speed></revolute><fixed id="fixed6"><frame><a idref="frame5"/><b idref="body5"/></frame><rotation><x>0</x><y>0</y><z>0</z></rotation><translation><x>0</x><y>-0.177</y><z>-0.185</z></translation></fixed><revolute id="joint5"><frame><a idref="body5"/><b idref="frame6"/></frame><axis><x>0</x><y>0</y><z>1</z></axis><max>360</max><min>-360</min><speed>360</speed></revolute><fixed id="fixed7"><frame><a idref="frame6"/><b idref="body6"/></frame><rotation><x>0</x><y>0</y><z>1</z></rotation><translation><x>0</x><y>0</y><z>0</z></translation></fixed><fixed id="fixed8"><frame><a idref="body6"/><b idref="frame7"/></frame><rotation><x>0</x><y>0</y><z>0</z></rotation><translation><x>0</x><y>0</y><z>0</z></translation></fixed></model></rlmdl>)";
}

DL_RobotContext::DL_RobotContext(const Handle(AIS_InteractiveContext)& theContext)
: m_context(theContext), m_paramR(0.0), m_paramH(0.0), m_paramL2(0.0), m_paramS(0.0),
  m_paramL3(0.0), m_paramD(0.0), m_paramW(0.0), m_isPose1(true)
{
    for (int i = 0; i < DL_ROBOT_JOINT_COUNT; ++i) { m_listJoints_angles[i] = 0.0; m_listJoints_angles0[i] = 0.0; }
}

DL_RobotContext::~DL_RobotContext() {}

double* DL_RobotContext::getPositions() { return m_listJoints_angles; }
const double* DL_RobotContext::getPositions() const { return m_listJoints_angles; }
int DL_RobotContext::getRodShapeCount() const { return DL_ROBOT_JOINT_COUNT + 1; }
Handle(AIS_Shape) DL_RobotContext::getRodShape(int theIndex) const { return (theIndex >= 0 && theIndex <= DL_ROBOT_JOINT_COUNT) ? m_listRods[theIndex] : nullptr; }
bool DL_RobotContext::isLoaded() const { return !m_context.IsNull() && m_rl_mdl_KinematicModel && !m_listRods[0].IsNull() && !m_listRods[DL_ROBOT_JOINT_COUNT].IsNull(); }

double DL_RobotContext::getPosition(int theJointIndex) const
{
    return (theJointIndex >= 1 && theJointIndex <= DL_ROBOT_JOINT_COUNT) ? m_listJoints_angles[theJointIndex - 1] : 0.0;
}

void DL_RobotContext::setPositions0(const double* theAngles)
{
    if (!theAngles) return;
    for (int i = 0; i < DL_ROBOT_JOINT_COUNT; ++i) m_listJoints_angles0[i] = theAngles[i];
}

void DL_RobotContext::setPositions(const double* theAngles)
{
    if (!theAngles) return;
    for (int i = 0; i < DL_ROBOT_JOINT_COUNT; ++i) m_listJoints_angles[i] = theAngles[i];
    forwardRobot();
    if (!m_context.IsNull()) m_context->UpdateCurrentViewer();
}

void DL_RobotContext::setTcp(const gp_Trsf& theTransform)
{
    m_tfTCP = theTransform;
    m_tfTCPInv = theTransform.Inverted();
    if (!m_ASTool.IsNull()) m_ASTool->SetLocalTransformation(m_tfTCP);
}

rl::math::Transform DL_RobotContext::trans(const gp_Trsf& theTransform) const
{
    rl::math::Transform aTransform = rl::math::Transform::Identity();
    for (int r = 0; r < 3; ++r) { for (int c = 0; c < 3; ++c) aTransform(r, c) = theTransform.Value(r + 1, c + 1); aTransform(r, 3) = theTransform.Value(r + 1, 4); }
    return aTransform;
}

gp_Trsf DL_RobotContext::trans(const rl::math::Transform& theTransform) const
{
    gp_Trsf aTransform;
    aTransform.SetValues(theTransform(0, 0), theTransform(0, 1), theTransform(0, 2), theTransform(0, 3),
                         theTransform(1, 0), theTransform(1, 1), theTransform(1, 2), theTransform(1, 3),
                         theTransform(2, 0), theTransform(2, 1), theTransform(2, 2), theTransform(2, 3));
    return aTransform;
}

void DL_RobotContext::loadMdlModel(const char* xmlFileName)
{
    if (!xmlFileName || '\0' == xmlFileName[0]) throw std::runtime_error("robot.xml path is empty");
    m_rl_mdl_IKSolver.reset(); m_rl_mdl_KinematicModel.reset(); m_rl_mdl_Model.reset();
    rl::mdl::XmlFactory aFactory;
    m_rl_mdl_Model.reset(aFactory.create(xmlFileName));
    rl::mdl::Kinematic* aKinematic = dynamic_cast<rl::mdl::Kinematic*>(m_rl_mdl_Model.get());
    if (!aKinematic) { m_rl_mdl_Model.reset(); throw std::runtime_error("RL model is not kinematic"); }
    m_rl_mdl_KinematicModel = std::shared_ptr<rl::mdl::Kinematic>(m_rl_mdl_Model, aKinematic);
    std::printf("RL Kinematics XML loaded successfully once.\n");
}

Handle(AIS_Shape) DL_RobotContext::loadStp(const char* stpFileName)
{
    STEPControl_Reader aReader;
    if (aReader.ReadFile(stpFileName) != IFSelect_RetDone) { std::printf("[Error] 无法读取文件: %s\n", stpFileName); return nullptr; }
    bool showFailsOnly = false;
    std::printf("\n>>> 模型加载检查 [%s]:\n", stpFileName);
    aReader.PrintCheckLoad(showFailsOnly, IFSelect_CountByItem);
    aReader.TransferRoots();
    aReader.PrintCheckTransfer(showFailsOnly, IFSelect_CountByItem);
    TopoDS_Shape aShape = aReader.OneShape();
    if (aShape.IsNull()) { std::printf("[Warning] 文件加载成功但未发现有效几何体\n"); return nullptr; }
    std::printf(">>> 模型加载成功: 找到 %d 个根对象，已合并显示。\n", aReader.NbRootsForTransfer());
    return new AIS_Shape(aShape);
}

Handle(AIS_Shape) DL_RobotContext::loadIges(const char* igesFileName)
{
    IGESControl_Reader aReader;
    if (aReader.ReadFile(igesFileName) != IFSelect_RetDone) return nullptr;
    aReader.TransferRoots();
    TopoDS_Shape aShape = aReader.OneShape();
    return aShape.IsNull() ? nullptr : new AIS_Shape(aShape);
}

bool DL_RobotContext::loadRobotXml()
{
    QFile aFile(m_robotXmlFileName);
    if (!aFile.open(QIODevice::ReadOnly | QIODevice::Text)) return false;
    m_robotXmlDocument.clear();
    bool ok = m_robotXmlDocument.setContent(&aFile);
    aFile.close();
    return ok;
}

double DL_RobotContext::getFixedVal(const char* theId, const char* theAxis) const
{
    QDomNodeList aNodes = m_robotXmlDocument.elementsByTagName("fixed");
    for (int i = 0; i < aNodes.count(); ++i) { QDomElement e = aNodes.at(i).toElement(); if (e.attribute("id") == QString::fromLatin1(theId)) return e.firstChildElement("translation").firstChildElement(QString::fromLatin1(theAxis)).text().toDouble(); }
    return 0.0;
}

gp_Dir DL_RobotContext::getJointDir(int theIndex) const
{
    QDomNodeList aNodes = m_robotXmlDocument.elementsByTagName("revolute");
    if (theIndex < 0 || theIndex >= aNodes.count()) return gp_Dir(0.0, 0.0, 1.0);
    QDomElement aAxis = aNodes.at(theIndex).toElement().firstChildElement("axis");
    double x = aAxis.firstChildElement("x").text().toDouble();
    double y = aAxis.firstChildElement("y").text().toDouble();
    double z = aAxis.firstChildElement("z").text().toDouble();
    return (0.0 == x && 0.0 == y && 0.0 == z) ? gp_Dir(0.0, 0.0, 1.0) : gp_Dir(x, y, z);
}

Handle(AIS_Trihedron) DL_RobotContext::createEndTrihedron(const gp_Pnt& thePoint, const gp_Dir& theNormal, const gp_Dir& theXAxis)
{
    Handle(Geom_Axis2Placement) anAxis = new Geom_Axis2Placement(gp_Ax2(thePoint, theNormal, theXAxis));
    Handle(AIS_Trihedron) aTrihedron = new AIS_Trihedron(anAxis);
    aTrihedron->SetDatumDisplayMode(Prs3d_DM_Shaded);
    aTrihedron->SetArrowColor(Prs3d_DatumParts_XAxis, Quantity_NOC_RED);
    aTrihedron->SetArrowColor(Prs3d_DatumParts_YAxis, Quantity_NOC_GREEN);
    aTrihedron->SetArrowColor(Prs3d_DatumParts_ZAxis, Quantity_NOC_BLUE);
    aTrihedron->SetColor(Quantity_NOC_BLUE);
    aTrihedron->SetXAxisColor(Quantity_NOC_RED);
    aTrihedron->SetYAxisColor(Quantity_NOC_GREEN);
    m_endTrihedron = aTrihedron;
    return aTrihedron;
}

QString DL_RobotContext::findSampleImage(const char* theFileName)
{
    QStringList aDirs;
    aDirs << QDir(QCoreApplication::applicationDirPath()).filePath("samples")
          << QDir(QCoreApplication::applicationDirPath()).filePath("../samples")
          << QDir(QCoreApplication::applicationDirPath()).filePath("../../samples")
          << QDir(QDir::currentPath()).filePath("win/bin/samples")
          << QDir(QDir::currentPath()).filePath("win/bind/samples");
    for (QStringList::const_iterator it = aDirs.constBegin(); it != aDirs.constEnd(); ++it) { QString aPath = QDir(*it).filePath(QString::fromLatin1(theFileName)); if (QFileInfo::exists(aPath)) return QDir::cleanPath(aPath); }
    return QString();
}

void DL_RobotContext::clearRobotPresentation()
{
    if (!m_context.IsNull())
    {
        AIS_ListOfInteractive aObjects;
        m_context->DisplayedObjects(aObjects);
        for (AIS_ListIteratorOfListOfInteractive it(aObjects); it.More(); it.Next())
        {
            if (it.Value()->IsKind(STANDARD_TYPE(AIS_Shape))) m_context->Remove(it.Value(), Standard_False);
        }
        if (!m_endTrihedron.IsNull()) m_context->Remove(m_endTrihedron, Standard_False);
        if (!m_traceLine.IsNull()) m_context->Remove(m_traceLine, Standard_False);
    }

    for (int i = 0; i <= DL_ROBOT_JOINT_COUNT; ++i) m_listRods[i].Nullify();
    for (int i = 0; i < DL_ROBOT_JOINT_COUNT; ++i) m_listJoints_angles[i] = 0.0;
    m_traceLine.Nullify();
    m_endTrihedron.Nullify();
    m_rl_mdl_IKSolver.reset();
    m_rl_mdl_KinematicModel.reset();
    m_rl_mdl_Model.reset();
}

void DL_RobotContext::updateTraceLine()
{
    if (m_context.IsNull() || m_endTrihedron.IsNull()) return;
    if (!m_traceLine.IsNull()) m_context->Remove(m_traceLine, Standard_False);
    const gp_Trsf& aTransform = m_endTrihedron->Transformation();
    gp_Pnt aWorldPoint = m_endTrihedron->Component()->Location().Transformed(aTransform);
    TopoDS_Edge anEdge = BRepBuilderAPI_MakeEdge(gp_Pnt(0.0, 0.0, 0.0), aWorldPoint);
    m_traceLine = new AIS_Shape(anEdge);
    m_traceLine->SetColor(Quantity_NOC_GREEN);
    m_traceLine->SetWidth(3.0);
    m_context->Display(m_traceLine, Standard_False);
}

void DL_RobotContext::loadAISShapes()
{
    if (m_robotXmlDocument.isNull()) throw std::runtime_error("robot.xml not loaded");

    for (int i = 0; i <= DL_ROBOT_JOINT_COUNT; ++i)
    {
        QString aStpPath = QDir(m_robotDirPath).filePath(QString("ROD_1_%1.stp").arg(i + 1));
        m_listRods[i] = loadStp(aStpPath.toLocal8Bit().constData());
        if (m_listRods[i].IsNull()) throw std::runtime_error(QString("load rod failed: %1").arg(aStpPath).toStdString());
    }

    for (int i = 0; i < DL_ROBOT_JOINT_COUNT; ++i) m_listRods[i]->AddChild(m_listRods[i + 1]);
    for (int i = 0; i <= DL_ROBOT_JOINT_COUNT; ++i) { m_context->SetDisplayMode(m_listRods[i], 1, Standard_False); m_context->Display(m_listRods[i], Standard_False); }

    m_paramR = getFixedVal("fixed2", "x") * 1000.0;
    m_paramH = getFixedVal("fixed2", "z") * 1000.0;
    m_paramL2 = getFixedVal("fixed3", "z") * 1000.0;
    m_paramS = getFixedVal("fixed4", "z") * 1000.0;
    m_paramL3 = getFixedVal("fixed5", "x") * 1000.0;
    m_paramD = getFixedVal("fixed6", "y") * 1000.0;
    double tx7 = getFixedVal("fixed7", "x");
    double tz7 = getFixedVal("fixed7", "z");
    m_isPose1 = std::abs(tx7) > 0.0001;
    m_paramW = (m_isPose1 ? tx7 : tz7) * 1000.0;

    std::printf("\n--- XML 姿态识别结果 ---\n");
    if (m_isPose1) std::printf(">> 检测到 [姿态一：手腕平直], W = %.3f (X轴偏移)\n", tx7);
    else std::printf(">> 检测到 [姿态二：手腕弯折], W = %.3f (Z轴偏移)\n", tz7);

    m_listJoints[0] = gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), getJointDir(0));
    m_listJoints[1] = gp_Ax1(gp_Pnt(m_paramR, 0.0, m_paramH), getJointDir(1));
    m_listJoints[2] = gp_Ax1(gp_Pnt(m_paramR, 0.0, m_paramH + m_paramL2), getJointDir(2));
    m_listJoints[3] = gp_Ax1(gp_Pnt(m_paramR, 0.0, m_paramH + m_paramL2 + m_paramS), getJointDir(3));
    m_listJoints[4] = gp_Ax1(gp_Pnt(m_paramR + m_paramL3, 0.0, m_paramH + m_paramL2 + m_paramS), getJointDir(4));
    if (m_isPose1) m_listJoints[5] = gp_Ax1(gp_Pnt(0.0, m_paramD, m_paramH + m_paramL2 + m_paramS), getJointDir(5));
    else m_listJoints[5] = gp_Ax1(gp_Pnt(m_paramR + m_paramL3, m_paramD, 0.0), getJointDir(5));

    gp_Pnt aTcpPoint;
    if (m_isPose1) aTcpPoint.SetCoord(m_paramR + m_paramL3 + m_paramW, m_paramD, m_paramH + m_paramL2 + m_paramS);
    else aTcpPoint.SetCoord(m_paramR + m_paramL3, m_paramD, m_paramH + m_paramL2 + m_paramS + m_paramW);
    Handle(AIS_Trihedron) aTrihedron = createEndTrihedron(aTcpPoint, gp_Dir(0.0, 0.0, 1.0), gp_Dir(1.0, 0.0, 0.0));
    m_listRods[DL_ROBOT_JOINT_COUNT]->AddChild(aTrihedron);
    m_context->Display(aTrihedron, Standard_False);

    if (!m_ASTool.IsNull())
    {
        m_ASTool->SetLocalTransformation(m_tfTCP);
        m_listRods[DL_ROBOT_JOINT_COUNT]->AddChild(m_ASTool);
        m_context->SetDisplayMode(m_ASTool, 1, Standard_False);
        m_context->Display(m_ASTool, Standard_False);
    }
}

void DL_RobotContext::loadAll()
{
    loadMdlModel(m_robotXmlFileName.toLocal8Bit().constData());
    loadAISShapes();
    resetRobot();
}

void DL_RobotContext::forwardRobot()
{
    if (!isLoaded()) return;
    for (int i = 0; i < DL_ROBOT_JOINT_COUNT; ++i)
    {
        gp_Trsf aTransform;
        aTransform.SetRotation(m_listJoints[i], m_listJoints_angles[i]);
        if (!m_listRods[i + 1].IsNull()) m_listRods[i + 1]->SetLocalTransformation(aTransform);
    }

    std::size_t dof = m_rl_mdl_KinematicModel->getDof();
    rl::math::Vector q(dof);
    for (std::size_t i = 0; i < dof; ++i) q(i) = m_listJoints_angles[i];
    m_rl_mdl_KinematicModel->setPosition(q);
    m_rl_mdl_KinematicModel->forwardPosition();
    if (!m_ASTool.IsNull()) m_ASTool->SetLocalTransformation(m_tfTCP);
    updateTraceLine();
}

void DL_RobotContext::resetRobot()
{
    for (int i = 0; i < DL_ROBOT_JOINT_COUNT; ++i) m_listJoints_angles[i] = m_listJoints_angles0[i];
    forwardRobot();
    if (!m_context.IsNull()) m_context->UpdateCurrentViewer();
}

bool DL_RobotContext::ikSolve(const rl::math::Transform& theTransform, double theAngles[], bool isFromTcp)
{
    if (!isLoaded() || !theAngles) return false;
    rl::math::Transform aGoal = isFromTcp ? theTransform * trans(m_tfTCPInv) : theTransform;
    if (!m_rl_mdl_KinematicModel->calculateInversePosition(aGoal, 0, 0.5)) return false;
    rl::math::Vector q = m_rl_mdl_KinematicModel->getPosition();
    for (int i = 0; i < DL_ROBOT_JOINT_COUNT; ++i) theAngles[i] = q(i);
    return true;
}

rl::math::Transform DL_RobotContext::forwardSolve(const double theAngles[], bool isToTcp)
{
    rl::math::Transform aTransform = rl::math::Transform::Identity();
    if (!isLoaded() || !theAngles) return aTransform;
    std::size_t dof = m_rl_mdl_KinematicModel->getDof();
    rl::math::Vector q(dof);
    for (std::size_t i = 0; i < dof; ++i) q(i) = theAngles[i];
    m_rl_mdl_KinematicModel->setPosition(q);
    m_rl_mdl_KinematicModel->forwardPosition();
    aTransform = m_rl_mdl_KinematicModel->getOperationalPosition(0);
    return isToTcp ? aTransform * trans(m_tfTCP) : aTransform;
}

int DL_RobotContext::loadRobotDynamic(QWidget* theParent)
{
    if (m_context.IsNull()) return 0;
    QString aDirPath = QFileDialog::getExistingDirectory(theParent, "choose files", m_robotDirPath);
    if (aDirPath.isEmpty()) return 0;
    return loadRobot(aDirPath, theParent);
}

int DL_RobotContext::loadRobot(const QString& theDirPath, QWidget* theParent)
{
    if (m_context.IsNull()) return 0;
    if (theDirPath.isEmpty()) return 0;

    m_robotDirPath = theDirPath;
    m_robotXmlFileName = QDir(theDirPath).filePath("robot.xml");
    if (!loadRobotXml()) { std::printf("错误：缺失或无法解析 robot.xml 文件。\n"); QMessageBox::warning(theParent, "Tips", "robot.xml missing or invalid."); return 0; }

    try
    {
        clearRobotPresentation();
        loadAll();
        if (!m_context.IsNull()) m_context->UpdateCurrentViewer();
        return DL_ROBOT_JOINT_COUNT;
    }
    catch (const std::exception& e)
    {
        clearRobotPresentation();
        std::printf("加载机器人失败: %s\n", e.what());
        QMessageBox::warning(theParent, "Tips", QString("Load robot failed: %1").arg(e.what()));
        return 0;
    }
}

void DL_RobotContext::moveJoint(int theIndex, int theForward)
{
    if (!isLoaded() || theIndex < 0 || theIndex >= DL_ROBOT_JOINT_COUNT) return;
    m_listJoints_angles[theIndex] += theForward * 5.0 * rl::math::DEG2RAD;
    forwardRobot();
    if (!m_context.IsNull()) m_context->UpdateCurrentViewer();
}

void DL_RobotContext::calcRobot()
{
    if (!isLoaded() || m_endTrihedron.IsNull()) return;
    std::size_t dof = m_rl_mdl_KinematicModel->getDof();

    std::cout << "================ 关节限位（单位：度） ================" << std::endl;
    for (std::size_t i = 0; i < dof; ++i)
    {
        const rl::mdl::Joint* aJoint = m_rl_mdl_KinematicModel->getJoint(i);
        std::cout << "Joint " << i << ": min = " << aJoint->min(0) * rl::math::RAD2DEG << " °, max = " << aJoint->max(0) * rl::math::RAD2DEG << " °" << std::endl;
    }
    std::cout << "=====================================================" << std::endl;

    rl::math::Vector q(dof), q0(dof), qinv(dof);
    for (std::size_t i = 0; i < dof; ++i) { q(i) = m_listJoints_angles[i]; q0(i) = m_listJoints_angles0[i]; }
    m_rl_mdl_KinematicModel->setPosition(q);
    m_rl_mdl_KinematicModel->forwardPosition();
    rl::math::Transform t = m_rl_mdl_KinematicModel->getOperationalPosition(0);
    std::cout << "rl依据xml计算末端坐标 (X,Y,Z): " << t.translation().x() << ", " << t.translation().y() << ", " << t.translation().z() << std::endl;

    gp_Pnt aWorldPoint = m_endTrihedron->Component()->Location().Transformed(m_endTrihedron->Transformation());
    std::cout << "OCC 真实末端坐标 (X,Y,Z): " << aWorldPoint.X() << ", " << aWorldPoint.Y() << ", " << aWorldPoint.Z() << std::endl;

    m_rl_mdl_KinematicModel->setPosition(q0);
    m_rl_mdl_KinematicModel->forwardPosition();

    double aAngles[DL_ROBOT_JOINT_COUNT] = {0.0};
    if (!ikSolve(t, aAngles)) { std::printf("错误：逆解计算失败，无法到达目标位姿。\n"); return; }

    for (std::size_t i = 0; i < dof; ++i) { m_listJoints_angles[i] = aAngles[i]; qinv(i) = aAngles[i]; }
    forwardRobot();
    if (!m_context.IsNull()) m_context->UpdateCurrentViewer();

    rl::math::Transform tinv = m_rl_mdl_KinematicModel->getOperationalPosition(0);
    std::cout << "================ 正逆解一致性验证 ================" << std::endl;
    std::cout << "初始目标关节角 (q):    " << q.transpose() << std::endl;
    std::cout << "逆解计算关节角 (qinv): " << qinv.transpose() << std::endl;
    std::cout << "\n正解矩阵 (t):\n" << t.matrix() << std::endl;
    std::cout << "\n逆解矩阵 (tinv):\n" << tinv.matrix() << std::endl;
}

void DL_RobotContext::disasRobot(QWidget* theParent)
{
    if (m_context.IsNull()) return;
    std::printf("--- 开始执行拆分流程 ---\n");

    QString aFileName = QFileDialog::getOpenFileName(theParent, "choose stp or step", QString(), "STEP Files (*.stp *.step)");
    if (aFileName.isEmpty()) return;
    splitStepFile(aFileName);
}

bool DL_RobotContext::splitStepFile(const QString& theFileName)
{
    if (m_context.IsNull() || theFileName.isEmpty()) return false;

    QString aFileName = theFileName;
    QString aSaveDir = QFileInfo(aFileName).absolutePath();
    STEPControl_Reader aReader;
    if (aReader.ReadFile(aFileName.toLocal8Bit().constData()) != IFSelect_RetDone) { std::printf("读取失败\n"); return false; }
    aReader.TransferRoots();

    TopoDS_Compound aWholeCompound;
    BRep_Builder aBuilder;
    aBuilder.MakeCompound(aWholeCompound);

    int aShapesNumber = aReader.NbShapes();
    Standard_Real anOffset = 500.0;
    for (int i = 1; i <= aShapesNumber; ++i)
    {
        TopoDS_Shape aRootShape = aReader.Shape(i);
        int aPartIndex = 0;
        for (TopoDS_Iterator it(aRootShape); it.More(); it.Next())
        {
            TopoDS_Shape aPart = it.Value();
            ++aPartIndex;
            QString anOutputName = QString("%1/ROD_%2_%3.stp").arg(aSaveDir).arg(i).arg(aPartIndex);

            STEPControl_Writer aWriter;
            aWriter.Transfer(aPart, STEPControl_AsIs);
            aWriter.Write(anOutputName.toLocal8Bit().constData());
            std::printf("零件已保存至: %s\n", anOutputName.toLocal8Bit().constData());

            Handle(AIS_Shape) anAisPart = new AIS_Shape(aPart);
            gp_Trsf aTransform;
            aTransform.SetTranslation(gp_Vec(0.0, anOffset, 0.0));
            anAisPart->SetLocalTransformation(aTransform);
            m_context->SetDisplayMode(anAisPart, 1, Standard_False);
            m_context->Display(anAisPart, Standard_False);
            anOffset += 500.0;
            aBuilder.Add(aWholeCompound, aPart);
        }
    }

    Handle(AIS_Shape) anAisWhole = new AIS_Shape(aWholeCompound);
    m_context->SetDisplayMode(anAisWhole, 1, Standard_False);
    m_context->Display(anAisWhole, Standard_True);
    std::printf("--- 拆分显示完成 ---\n");
    return true;
}

void DL_RobotContext::loadTool(const char* modelFileName)
{
    if (!modelFileName) return;
    QString aFileName = QString::fromLocal8Bit(modelFileName);
    QString aSuffix = QFileInfo(aFileName).suffix().toLower();
    m_ASTool = ("igs" == aSuffix || "iges" == aSuffix) ? loadIges(modelFileName) : loadStp(modelFileName);
    if (m_ASTool.IsNull()) return;
    m_ASTool->SetLocalTransformation(m_tfTCP);
    if (!m_listRods[DL_ROBOT_JOINT_COUNT].IsNull()) m_listRods[DL_ROBOT_JOINT_COUNT]->AddChild(m_ASTool);
    if (!m_context.IsNull()) { m_context->SetDisplayMode(m_ASTool, 1, Standard_False); m_context->Display(m_ASTool, Standard_False); m_context->UpdateCurrentViewer(); }
}

void DL_RobotContext::writeRobotXml(QWidget* theParent)
{
    QDialog aDialog(theParent);
    aDialog.setWindowTitle("Write Robot Xml");
    QHBoxLayout* aMainLayout = new QHBoxLayout(&aDialog);

    QVBoxLayout* aLeftLayout = new QVBoxLayout();
    QLabel* aImageLabel = new QLabel();
    QString aPose1Image = findSampleImage("robotXml.png");
    QString aPose2Image = findSampleImage("down.png");
    auto updateImage = [aImageLabel, aPose1Image, aPose2Image](bool isPose1)
    {
        QString aPath = isPose1 ? aPose1Image : aPose2Image;
        if (aPath.isEmpty()) aImageLabel->clear();
        else aImageLabel->setPixmap(QPixmap(aPath).scaled(350, 450, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    };

    QRadioButton* aPose1Button = new QRadioButton("Case1:Straight (W in X)");
    QRadioButton* aPose2Button = new QRadioButton("Case2:Bend (W in Z)");
    aPose1Button->setChecked(true);
    updateImage(true);
    QObject::connect(aPose1Button, &QRadioButton::toggled, [&updateImage](bool checked) { updateImage(checked); });

    aLeftLayout->addWidget(aImageLabel);
    aLeftLayout->addWidget(aPose1Button);
    aLeftLayout->addWidget(aPose2Button);
    aLeftLayout->addStretch();
    aMainLayout->addLayout(aLeftLayout);

    QGridLayout* aGrid = new QGridLayout();
    aGrid->setHorizontalSpacing(15);
    aGrid->setVerticalSpacing(10);
    QLineEdit* aParamEdits[7] = {nullptr};
    QStringList aParamLabels = QStringList() << "R" << "H" << "L2" << "S" << "L3" << "D" << "W";
    QStringList aParamDefaults = QStringList() << "0" << "0.278" << "0.380" << "0" << "0.370" << "-0.125" << "-0.135";
    for (int i = 0; i < aParamLabels.count(); ++i)
    {
        QLabel* aLabel = new QLabel(aParamLabels[i]);
        aLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        aGrid->addWidget(aLabel, i, 0);
        aParamEdits[i] = new QLineEdit(aParamDefaults[i]);
        aParamEdits[i]->setAlignment(Qt::AlignCenter);
        aGrid->addWidget(aParamEdits[i], i, 1);
    }
    aGrid->setColumnMinimumWidth(2, 20);

    QLabel* aMinTitle = new QLabel("Min(°)");
    QLabel* aMaxTitle = new QLabel("Max(°)");
    QLabel* aSpeedTitle = new QLabel("MaxSpeed");
    aMinTitle->setAlignment(Qt::AlignCenter);
    aMaxTitle->setAlignment(Qt::AlignCenter);
    aSpeedTitle->setAlignment(Qt::AlignCenter);
    aGrid->addWidget(aMinTitle, 0, 4);
    aGrid->addWidget(aMaxTitle, 0, 5);
    aGrid->addWidget(aSpeedTitle, 0, 6);

    QLineEdit* aMinEdits[DL_ROBOT_JOINT_COUNT] = {nullptr};
    QLineEdit* aMaxEdits[DL_ROBOT_JOINT_COUNT] = {nullptr};
    QLineEdit* aSpeedEdits[DL_ROBOT_JOINT_COUNT] = {nullptr};
    for (int i = 0; i < DL_ROBOT_JOINT_COUNT; ++i)
    {
        QLabel* aJointLabel = new QLabel(QString("Joint %1:").arg(i));
        aJointLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        aGrid->addWidget(aJointLabel, i + 1, 3);

        aMinEdits[i] = new QLineEdit("-360");
        aMaxEdits[i] = new QLineEdit("360");
        aSpeedEdits[i] = new QLineEdit("0");
        aMinEdits[i]->setAlignment(Qt::AlignCenter);
        aMaxEdits[i]->setAlignment(Qt::AlignCenter);
        aSpeedEdits[i]->setAlignment(Qt::AlignCenter);
        aGrid->addWidget(aMinEdits[i], i + 1, 4);
        aGrid->addWidget(aMaxEdits[i], i + 1, 5);
        aGrid->addWidget(aSpeedEdits[i], i + 1, 6);
    }

    QPushButton* aCreateButton = new QPushButton("Create robot.xml");
    aGrid->addWidget(aCreateButton, 8, 0, 1, 7);
    aMainLayout->addLayout(aGrid);

    QObject::connect(aCreateButton, &QPushButton::clicked, [&]()
    {
        QDomDocument aDocument;
        if (!aDocument.setContent(QString::fromUtf8(ROBOT_XML_TEMPLATE))) return;

        auto updateText = [&aDocument](QDomElement theParent, const QString& theTag, const QString& theValue)
        {
            QDomElement aNode = theParent.firstChildElement(theTag);
            if (aNode.isNull()) { aNode = aDocument.createElement(theTag); theParent.appendChild(aNode); }
            if (aNode.firstChild().isNull()) aNode.appendChild(aDocument.createTextNode(theValue));
            else aNode.firstChild().setNodeValue(theValue);
        };

        auto setFixed = [&aDocument, &updateText](const QString& theId, const QString& theX, const QString& theY, const QString& theZ)
        {
            QDomNodeList aNodes = aDocument.elementsByTagName("fixed");
            for (int i = 0; i < aNodes.count(); ++i)
            {
                QDomElement e = aNodes.at(i).toElement();
                if (e.attribute("id") != theId) continue;
                QDomElement t = e.firstChildElement("translation");
                updateText(t, "x", theX); updateText(t, "y", theY); updateText(t, "z", theZ);
                return;
            }
        };

        setFixed("fixed2", aParamEdits[0]->text(), "0", aParamEdits[1]->text());
        setFixed("fixed3", "0", "0", aParamEdits[2]->text());
        setFixed("fixed4", "0", "0", aParamEdits[3]->text());
        setFixed("fixed5", aParamEdits[4]->text(), "0", "0");
        setFixed("fixed6", "0", aParamEdits[5]->text(), "0");
        bool isPose1 = aPose1Button->isChecked();
        if (isPose1) setFixed("fixed7", aParamEdits[6]->text(), "0", "0");
        else setFixed("fixed7", "0", "0", aParamEdits[6]->text());

        QStringList aAxisX = QStringList() << "0" << "0" << "0" << "1" << "0" << "1";
        QStringList aAxisY = QStringList() << "0" << "1" << "1" << "0" << "1" << "0";
        QStringList aAxisZ = QStringList() << "1" << "0" << "0" << "0" << "0" << "0";
        if (!isPose1) { aAxisX[5] = "0"; aAxisZ[5] = "1"; }

        QDomNodeList aJointNodes = aDocument.elementsByTagName("revolute");
        for (int i = 0; i < DL_ROBOT_JOINT_COUNT; ++i)
        {
            QDomElement aJoint = aJointNodes.at(i).toElement();
            QDomElement aAxis = aJoint.firstChildElement("axis");
            updateText(aAxis, "x", aAxisX[i]); updateText(aAxis, "y", aAxisY[i]); updateText(aAxis, "z", aAxisZ[i]);
            updateText(aJoint, "min", aMinEdits[i]->text());
            updateText(aJoint, "max", aMaxEdits[i]->text());
            updateText(aJoint, "speed", aSpeedEdits[i]->text());
        }

        QString aSavePath = QFileDialog::getExistingDirectory(&aDialog, "Save to...");
        if (aSavePath.isEmpty()) return;

        QFile aFile(QDir(aSavePath).filePath("robot.xml"));
        if (!aFile.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) { QMessageBox::warning(&aDialog, "Tips", "robot.xml can not be written."); return; }
        QTextStream aStream(&aFile);
        aStream.setCodec("UTF-8");
        aStream << aDocument.toString();
        aFile.close();
        QMessageBox::information(&aDialog, "Done", "robot.xml created!");
        aDialog.accept();
    });

    aDialog.exec();
}
