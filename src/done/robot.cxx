#include <TopTools_HSequenceOfShape.hxx>
#include <STEPControl_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shape.hxx>
#include <TCollection_AsciiString.hxx>
#include <BRep_Builder.hxx>
#include <AIS_Shape.hxx>
#include <Standard_Handle.hxx>

#include <QMdiArea>
#include <QMdiSubWindow>

#include "DocumentTut.h"
#include <gp_Ax2.hxx>
#include <Geom_Axis2Placement.hxx>
#include <AIS_Trihedron.hxx>
#include <Prs3d_DatumAspect.hxx>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <rl/kin/Kinematics.h>
#include <rl/math/Transform.h>
#include <rl/math/Unit.h>
//正反解

#include <rl/mdl/Kinematic.h>
#include <rl/mdl/XmlFactory.h>
//打印关节限位
#include <rl/mdl/Joint.h>
//打印当前末端坐标
#include <rl/math/Transform.h>
//外部加载机器人
#include <QFileDialog>
#include <QSettings>
#include <QDir>
#include <QFileInfo>
#include <QStringList>
//load rod
#include <AIS_InteractiveContext.hxx>
#include <STEPControl_Writer.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopExp_Explorer.hxx>
#include <gp_Trsf.hxx>
#include <stdio.h>

#include <QMessageBox>
#include <STEPControl_Writer.hxx>
#include <TopoDS_Iterator.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
//writeXml
#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QGridLayout>
#include <QPushButton>
#include <QFile>
#include <QTextStream>
#include <QComboBox>
#include <QDomDocument>
#include <QDomElement>
#include <QDomNodeList>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPixmap>
#include <QDebug>
// 布局和按钮相关的头文件
#include <QRadioButton>
#include <QPushButton>
// 数学常数 M_PI
#include <QtMath>
//线
#include <BRepBuilderAPI_MakeEdge.hxx>


static const char* ROBOT_XML_TEMPLATE = R"(<?xml version="1.0" encoding="UTF-8"?>
                                        <rlmdl xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="rlmdl.xsd">
                                        <model>
                                        <manufacturer>Huashu</manufacturer>
                                        <name>HSR-CR630-1750</name>
                                        <world id="world">
                                        <rotation>
                                        <x>0</x>
                                        <y>0</y>
                                        <z>0</z>
                                        </rotation>
                                        <translation>
                                        <x>0</x>
                                        <y>0</y>
                                        <z>0</z>
                                        </translation>
                                        <g>
                                        <x>0</x>
                                        <y>0</y>
                                        <z>9.86055</z>
                                        </g>
                                        </world>
                                        <body id="body0">
                                        <ignore/>
                                        <ignore idref="body1"/>
                                        </body>
                                        <frame id="frame0"/>
                                        <frame id="frame1"/>
                                        <body id="body1">
                                        <ignore idref="body0"/>
                                        <ignore idref="body2"/>
                                        </body>
                                        <frame id="frame2"/>
                                        <body id="body2">
                                        <ignore idref="body1"/>
                                        <ignore idref="body3"/>
                                        <ignore idref="body4"/>
                                        </body>
                                        <frame id="frame3"/>
                                        <body id="body3">
                                        <ignore idref="body2"/>
                                        <ignore idref="body4"/>
                                        </body>
                                        <frame id="frame4"/>
                                        <body id="body4">
                                        <ignore idref="body2"/>
                                        <ignore idref="body3"/>
                                        <ignore idref="body5"/>
                                        <ignore idref="body6"/>
                                        </body>
                                        <frame id="frame5"/>
                                        <body id="body5">
                                        <ignore idref="body4"/>
                                        <ignore idref="body6"/>
                                        </body>
                                        <frame id="frame6"/>
                                        <body id="body6">
                                        <ignore idref="body4"/>
                                        <ignore idref="body5"/>
                                        </body>
                                        <frame id="frame7"/>
                                        <fixed id="fixed0">
                                        <frame>
                                        <a idref="world"/>
                                        <b idref="body0"/>
                                        </frame>
                                        <rotation>
                                        <x>0</x>
                                        <y>0</y>
                                        <z>0</z>
                                        </rotation>
                                        <translation>
                                        <x>0</x>
                                        <y>0</y>
                                        <z>0</z>
                                        </translation>
                                        </fixed>
                                        <fixed id="fixed1">
                                        <frame>
                                        <a idref="body0"/>
                                        <b idref="frame0"/>
                                        </frame>
                                        <rotation>
                                        <x>0</x>
                                        <y>0</y>
                                        <z>0</z>
                                        </rotation>
                                        <translation>
                                        <x>0</x>
                                        <y>0</y>
                                        <z>0</z>
                                        </translation>
                                        </fixed>
                                        <revolute id="joint0">
                                        <frame>
                                        <a idref="frame0"/>
                                        <b idref="frame1"/>
                                        </frame>
                                        <axis>
                                        <x>0</x>
                                        <y>0</y>
                                        <z>1</z>
                                        </axis>
                                        <max>360</max>
                                        <min>-360</min>
                                        <speed>187</speed>
                                        </revolute>
                                        <fixed id="fixed2">
                                        <frame>
                                        <a idref="frame1"/>
                                        <b idref="body1"/>
                                        </frame>
                                        <rotation>
                                        <x>0</x>
                                        <y>0</y>
                                        <z>0</z>
                                        </rotation>
                                        <translation>
                                        <x>0</x>
                                        <y>0</y>
                                        <z>0.5015</z>
                                        </translation>
                                        </fixed>
                                        <revolute id="joint1">
                                        <frame>
                                        <a idref="body1"/>
                                        <b idref="frame2"/>
                                        </frame>
                                        <axis>
                                        <x>0</x>
                                        <y>1</y>
                                        <z>0</z>
                                        </axis>
                                        <max>360</max>
                                        <min>-360</min>
                                        <speed>160</speed>
                                        </revolute>
                                        <fixed id="fixed3">
                                        <frame>
                                        <a idref="frame2"/>
                                        <b idref="body2"/>
                                        </frame>
                                        <rotation>
                                        <x>0</x>
                                        <y>0</y>
                                        <z>0</z>
                                        </rotation>
                                        <translation>
                                        <x>0</x>
                                        <y>0</y>
                                        <z>0.950</z>
                                        </translation>
                                        </fixed>
                                        <revolute id="joint2">
                                        <frame>
                                        <a idref="body2"/>
                                        <b idref="frame3"/>
                                        </frame>
                                        <axis>
                                        <x>0</x>
                                        <y>1</y>
                                        <z>0</z>
                                        </axis>
                                        <max>360</max>
                                        <min>-360</min>
                                        <speed>180</speed>
                                        </revolute>
                                        <fixed id="fixed4">
                                        <frame>
                                        <a idref="frame3"/>
                                        <b idref="body3"/>
                                        </frame>
                                        <rotation>
                                        <x>0</x>
                                        <y>0</y>
                                        <z>0</z>
                                        </rotation>
                                        <translation>
                                        <x>0</x>
                                        <y>0</y>
                                        <z>0</z>
                                        </translation>
                                        </fixed>
                                        <revolute id="joint3">
                                        <frame>
                                        <a idref="body3"/>
                                        <b idref="frame4"/>
                                        </frame>
                                        <axis>
                                        <x>1</x>
                                        <y>0</y>
                                        <z>0</z>
                                        </axis>
                                        <max>360</max>
                                        <min>-360</min>
                                        <speed>260</speed>
                                        </revolute>
                                        <fixed id="fixed5">
                                        <frame>
                                        <a idref="frame4"/>
                                        <b idref="body4"/>
                                        </frame>
                                        <rotation>
                                        <x>0</x>
                                        <y>0</y>
                                        <z>0</z>
                                        </rotation>
                                        <translation>
                                        <x>0.615</x>
                                        <y>0</y>
                                        <z>0</z>
                                        </translation>
                                        </fixed>
                                        <revolute id="joint4">
                                        <frame>
                                        <a idref="body4"/>
                                        <b idref="frame5"/>
                                        </frame>
                                        <axis>
                                        <x>0</x>
                                        <y>1</y>
                                        <z>0</z>
                                        </axis>
                                        <max>360</max>
                                        <min>-360</min>
                                        <speed>230</speed>
                                        </revolute>
                                        <fixed id="fixed6">
                                        <frame>
                                        <a idref="frame5"/>
                                        <b idref="body5"/>
                                        </frame>
                                        <rotation>
                                        <x>0</x>
                                        <y>0</y>
                                        <z>0</z>
                                        </rotation>
                                        <translation>
                                        <x>0</x>
                                        <y>-0.177</y>
                                        <z>-0.185</z>
                                        </translation>
                                        </fixed>
                                        <revolute id="joint5">
                                        <frame>
                                        <a idref="body5"/>
                                        <b idref="frame6"/>
                                        </frame>
                                        <axis>
                                        <x>0</x>
                                        <y>0</y>
                                        <z>1</z>
                                        </axis>
                                        <max>360</max>
                                        <min>-360</min>
                                        <speed>360</speed>
                                        </revolute>
                                        <fixed id="fixed7">
                                        <frame>
                                        <a idref="frame6"/>
                                        <b idref="body6"/>
                                        </frame>
                                        <rotation>
                                        <x>0</x>
                                        <y>0</y>
                                        <z>1</z>
                                        </rotation>
                                        <translation>
                                        <x>0</x>
                                        <y>0</y>
                                        <z>0</z>
                                        </translation>
                                        </fixed>
                                        <fixed id="fixed8">
                                        <frame>
                                        <a idref="body6"/>
                                        <b idref="frame7"/>
                                        </frame>
                                        <rotation>
                                        <x>0</x>
                                        <y>0</y>
                                        <z>0</z>
                                        </rotation>
                                        <translation>
                                        <x>0</x>
                                        <y>0</y>
                                        <z>0</z>
                                        </translation>
                                        </fixed>
                                        </model>
                                        </rlmdl>)";


//
//static Handle(AIS_Trihedron) g_endTrihedron;
//
Handle(AIS_Trihedron)  createEndTrihedron(const gp_Pnt &P, const gp_Dir &N, const gp_Dir &Vx);
#define JOINT_N 6

Handle(AIS_Shape) 	Rods[JOINT_N+1];
gp_Ax1			  	Joints[JOINT_N];
double  			Joint_angles[JOINT_N] = {0.0};
Handle(AIS_Trihedron) g_endTrihedron;
static std::shared_ptr<rl::mdl::Model> g_rlModel = nullptr;
static rl::mdl::Kinematic* g_rlKinematics = nullptr;

Handle(AIS_Shape) g_traceLine = NULL; // 用于保存追踪线的全局句柄

//实时更新连线的封装函数
void updateTraceLine(Handle(AIS_InteractiveContext) aCtx)
{
    if (g_endTrihedron.IsNull()) return;

    // calcRobot() 中经过验证的求末端坐标逻辑
    const gp_Trsf& T = g_endTrihedron->Transformation();
    gp_Pnt P_initial = g_endTrihedron->Component()->Location();
    gp_Pnt P_world = P_initial.Transformed(T);

    // 如果画面中已经有线，先从上下文移除
    if (!g_traceLine.IsNull()) {
        aCtx->Remove(g_traceLine, Standard_False);
    }

    // 从绝对原点 (0,0,0) 到当前真实末端坐标创建拓扑线
    TopoDS_Edge aEdge = BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), P_world);
    g_traceLine = new AIS_Shape(aEdge);

    // 设置高亮颜色和线宽方便调试
    g_traceLine->SetColor(Quantity_NOC_GREEN);
    g_traceLine->SetWidth(3.0);

    // 显示新线（不立即刷新，等待外部统一刷新Viewer）
    aCtx->Display(g_traceLine, Standard_False);
}

void disasRobot(QMdiArea* ws)
{
    printf("--- 开始执行拆分流程 ---\n");
    MDIWindow* activeWin = qobject_cast<MDIWindow*>(ws->activeSubWindow()->widget());
    if (!activeWin) {
        QMessageBox::warning(nullptr, "Tips", "Please open a occ3D Window！");
        return;
    }
    Handle(AIS_InteractiveContext) aCtx = activeWin->getDocument()->getContext_AIS_InteractiveContext();

    // 1. 选择文件
    QString fileName = QFileDialog::getOpenFileName(nullptr, "choose stp or step", "", "STEP Files (*.stp *.step)");
    if (fileName.isEmpty()) return;

    // 获取该文件所在的目录，保存到原文件夹
    QFileInfo fileInfo(fileName);
    QString saveDir = fileInfo.absolutePath();

    // 2. 读取模型
    STEPControl_Reader aReader;
    if (aReader.ReadFile(fileName.toLocal8Bit().data()) != IFSelect_RetDone) {
        printf("读取失败\n");
        return;
    }

    aReader.TransferRoots();
    TopoDS_Compound aWholeCompound;
    BRep_Builder aBuilder;
    aBuilder.MakeCompound(aWholeCompound);

    int aShapesNumber = aReader.NbShapes();
    Standard_Real offset = 500.0; // 爆炸位移

    for (int i = 1; i <= aShapesNumber; i++) {
        TopoDS_Shape aRootShape = aReader.Shape(i);
        int partIdx = 0;

        // 遍历所有 SOLID 零件
        for (TopoDS_Iterator it(aRootShape); it.More(); it.Next()) {

            // 2. 获取当前的子节点
            TopoDS_Shape aPart = it.Value();

            partIdx++;

            // 保存到原文件夹的绝对路径
            QString outName = QString("%1/ROD_%2_%3.stp").arg(saveDir).arg(i).arg(partIdx);

            STEPControl_Writer aWriter;
            aWriter.Transfer(aPart, STEPControl_AsIs);
            aWriter.Write(outName.toLocal8Bit().data());
            printf("零件已保存至: %s\n", outName.toLocal8Bit().data());

            // 爆炸显示
            Handle(AIS_Shape) aisPart = new AIS_Shape(aPart);
            gp_Trsf trsf;
            trsf.SetTranslation(gp_Vec(0, offset, 0));
            aisPart->SetLocalTransformation(trsf);


            aCtx->SetDisplayMode(aisPart, 1, Standard_False);
            aCtx->Display(aisPart, Standard_False);
            offset += 500.0;

            // 同时加入总组合体
            aBuilder.Add(aWholeCompound, aPart);
        }
    }

    // 3. 原位显示总模型
    Handle(AIS_Shape) aisWhole = new AIS_Shape(aWholeCompound);


    aCtx->SetDisplayMode(aisWhole, 1, Standard_False);
    aCtx->Display(aisWhole, Standard_True);
    printf("--- 拆分显示完成 ---\n");
}



void writeRobotXml(QMdiArea* ws)
{

    QDialog dialog(ws);
    dialog.setWindowTitle("Write Robot Xml");
    QHBoxLayout* mainLayout = new QHBoxLayout(&dialog);

    // 左侧图片与状态切换
    QVBoxLayout* leftLayout = new QVBoxLayout();
    QLabel* imgLabel = new QLabel;

    QString pathPose1 = "D:/OCC/GC-OCC/GCDL-Weld/Tutorial/win/bin/samples/robotXml.png";
    QString pathPose2 = "D:/OCC/GC-OCC/GCDL-Weld/Tutorial/win/bin/samples/down.png";

    // 初始化显示姿态一图片
    imgLabel->setPixmap(QPixmap(pathPose1).scaled(350, 450, Qt::KeepAspectRatio));

    QRadioButton* rbPose1 = new QRadioButton("Case1:Straight (W in X)");
    QRadioButton* rbPose2 = new QRadioButton("Case2:Bend (W in Z)");
    rbPose1->setChecked(true); // 默认方案一

    // 添加图片切换逻辑
    QObject::connect(rbPose1, &QRadioButton::toggled, [=](bool checked){
        if(checked) {
            imgLabel->setPixmap(QPixmap(pathPose1).scaled(350, 450, Qt::KeepAspectRatio));
        } else {
            imgLabel->setPixmap(QPixmap(pathPose2).scaled(350, 450, Qt::KeepAspectRatio));
        }
    });

    leftLayout->addWidget(imgLabel);
    leftLayout->addWidget(rbPose1);
    leftLayout->addWidget(rbPose2);
    leftLayout->addStretch();
    mainLayout->addLayout(leftLayout);

    // 右侧输入表单
    QGridLayout* grid = new QGridLayout();
    grid->setHorizontalSpacing(15); // 增加列之间的间距
    grid->setVerticalSpacing(10);   // 增加行之间的间距
    // 基础尺寸 (7个)
    QLineEdit* leParams[7];
    QStringList paramLabels = {"R", "H", "L2", "S", "L3", "D", "W"};
    QStringList paramDefs = {"0", "0.278", "0.380", "0", "0.370", "-0.125", "-0.135"};
    for(int i=0; i<7; ++i) {
        QLabel* lbl = new QLabel(paramLabels[i]);
        lbl->setAlignment(Qt::AlignRight | Qt::AlignVCenter); // 标签靠右对齐，贴近输入框
        grid->addWidget(lbl, i, 0);

        leParams[i] = new QLineEdit(paramDefs[i]);
        leParams[i]->setAlignment(Qt::AlignCenter); // 文本框内容居中
        grid->addWidget(leParams[i], i, 1);
    }
    grid->setColumnMinimumWidth(2, 20);//竖向分割左右尺寸区与关节参数区
    // 18个关节限位输入框 (6关节 x 3参数)
    QLabel* titleMin = new QLabel("Min(°)");
    QLabel* titleMax = new QLabel("Max(°)");
    QLabel* titleSpd = new QLabel("MaxSpeed");
    titleMin->setAlignment(Qt::AlignCenter);
    titleMax->setAlignment(Qt::AlignCenter);
    titleSpd->setAlignment(Qt::AlignCenter);

    grid->addWidget(titleMin, 0, 4);//addWidget(控件, 起始行, 起始列, 行跨度, 列跨度)
    grid->addWidget(titleMax, 0, 5);
    grid->addWidget(titleSpd, 0, 6);
    QLineEdit* leMin[6], *leMax[6], *leSpeed[6];
    for(int i=0; i<6; ++i) {
        QLabel* jLbl = new QLabel(QString("Joint %1:").arg(i));
        jLbl->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        grid->addWidget(jLbl, i + 1, 3);

        leMin[i] = new QLineEdit("-360");
        leMax[i] = new QLineEdit("360");
        leSpeed[i] = new QLineEdit("0");

        // 让输入框的文字也全部居中显示
        leMin[i]->setAlignment(Qt::AlignCenter);
        leMax[i]->setAlignment(Qt::AlignCenter);
        leSpeed[i]->setAlignment(Qt::AlignCenter);

        grid->addWidget(leMin[i], i + 1, 4);
        grid->addWidget(leMax[i], i + 1, 5);
        grid->addWidget(leSpeed[i], i + 1, 6);
    }

    QPushButton* btnOk = new QPushButton("Create robot.xml");
    grid->addWidget(btnOk, 8, 0, 1, 6);
    mainLayout->addLayout(grid);

    QObject::connect(btnOk, &QPushButton::clicked, [&]() {
        QDomDocument doc;
        if (!doc.setContent(QString::fromUtf8(ROBOT_XML_TEMPLATE))) return;

        auto updateText = [&](QDomElement parent, QString tag, QString val) {
            QDomElement c = parent.firstChildElement(tag);
            if (c.isNull()) { c = doc.createElement(tag); parent.appendChild(c); }
            c.replaceChild(doc.createTextNode(val), c.firstChild());
        };

        auto setFixed = [&](QString id, QString x, QString y, QString z) {
            QDomNodeList list = doc.elementsByTagName("fixed");
            for(int i=0; i<list.count(); ++i) {
                QDomElement e = list.at(i).toElement();
                if(e.attribute("id") == id) {
                    QDomElement t = e.firstChildElement("translation");
                    updateText(t, "x", x); updateText(t, "y", y); updateText(t, "z", z);
                }
            }
        };

        // 1. 写入尺寸参数
        setFixed("fixed2", leParams[0]->text(), "0", leParams[1]->text()); // R, H
        setFixed("fixed3", "0", "0", leParams[2]->text()); // L2
        setFixed("fixed4", "0", "0", leParams[3]->text()); // S
        setFixed("fixed5", leParams[4]->text(), "0", "0"); // L3
        setFixed("fixed6", "0", leParams[5]->text(), "0"); // D

        bool isPose1 = rbPose1->isChecked();
        if(isPose1) setFixed("fixed7", leParams[6]->text(), "0", "0"); // Pose1: W in X
        else        setFixed("fixed7", "0", "0", leParams[6]->text()); // Pose2: W in Z

        // 2. 写入 18 个限位参数与轴向硬编码
        QStringList axisX = {"0","0","0","1","0","1"}; // 姿态一最后是 X
        QStringList axisY = {"0","1","1","0","1","0"};
        QStringList axisZ = {"1","0","0","0","0","0"};
        if(!isPose1) { axisX[5]="0"; axisZ[5]="1"; }   // 姿态二最后是 Z

        QDomNodeList jList = doc.elementsByTagName("revolute");
        for(int i=0; i<6; ++i) {
            QDomElement rev = jList.at(i).toElement();
            QDomElement ax = rev.firstChildElement("axis");
            updateText(ax, "x", axisX[i]);
            updateText(ax, "y", axisY[i]);
            updateText(ax, "z", axisZ[i]);

            // 写入 Min/Max/Speed (弧度转换)
            updateText(rev, "min", QString::number(leMin[i]->text().toDouble()));
            updateText(rev, "max", QString::number(leMax[i]->text().toDouble()));
            updateText(rev, "speed", leSpeed[i]->text());
        }

        // 保存文件 (逻辑保持不变)
        QString savePath = QFileDialog::getExistingDirectory(nullptr, "Save to...");
        if(!savePath.isEmpty()) {
            QFile f(QDir(savePath).filePath("robot.xml"));
            if(f.open(QIODevice::WriteOnly)) {
                QTextStream(&f) << doc.toString();
                f.close();
                QMessageBox::information(nullptr, "Done", "robot.xml created!");
                dialog.accept();
            }
        }
    });

    dialog.exec();




}


Handle(AIS_Shape) loadRodModel(const char* theFileName)
{
    STEPControl_Reader aReader;
    const IFSelect_ReturnStatus aStatus = aReader.ReadFile(theFileName);

    if (aStatus != IFSelect_RetDone) {
        printf("[Error] 无法读取文件: %s\n", theFileName);
        return NULL;
    }

    // --- 诊断信息打印 ---
    // 设置为 mode 1 (IFSelect_CountByItem)，快速查看模型是否有严重错误
    bool showFailsOnly = false;
    printf("\n>>> 模型加载检查 [%s]:\n", theFileName);
    aReader.PrintCheckLoad(showFailsOnly, IFSelect_CountByItem);

    // 执行转换逻辑（只转换根节点，避免重复提取）
    aReader.TransferRoots();

    // 转换后的转换结果检查
    aReader.PrintCheckTransfer(showFailsOnly, IFSelect_CountByItem);

 // OneShape() 会自动处理 Compound 逻辑，完全替代你之前的 BRep_Builder 手动循环
    TopoDS_Shape aShape = aReader.OneShape();

    if (aShape.IsNull()) {
        printf("[Warning] 文件加载成功但未发现有效几何体\n");
        return NULL;
    }

    printf(">>> 模型加载成功: 找到 %d 个根对象，已合并显示。\n", aReader.NbRootsForTransfer());

    return new AIS_Shape(aShape);
}


//全局函数加载xml，load载入机器人时调用一次
bool initRLKinematics(const char* xmlPath)
{

    try
    {
        // 如果已经加载了旧模型，先清理掉，防止内存冲突报错
        if (g_rlModel != nullptr)
        {
            g_rlModel.reset();
            g_rlKinematics = nullptr;
        }
        static rl::mdl::XmlFactory factory;
        g_rlModel = std::shared_ptr<rl::mdl::Model>(factory.create(xmlPath));
        g_rlKinematics = dynamic_cast<rl::mdl::Kinematic*>(g_rlModel.get());

        if (g_rlKinematics) {
            printf("RL Kinematics XML loaded successfully once.\n");
            return true;
        }
    } catch (const std::exception& e) {
        printf("RL XML Load Error: %s\n", e.what());
    }
    return false;
}

void resetRobot(QMdiArea* ws)
{//复位代码
    for(int i=0;i<JOINT_N;i++)
    {
        Joint_angles[i]=0* rl::math::DEG2RAD;
        gp_Trsf aRodTrsf;
        aRodTrsf.SetRotation(Joints[i],Joint_angles[i]);
        Rods[i+1]->SetLocalTransformation(aRodTrsf);
    }

    DocumentCommon* doc = qobject_cast<MDIWindow*>(
                ws->activeSubWindow()->widget() )->getDocument();
    Handle(AIS_InteractiveContext) aCtx = doc->getContext_AIS_InteractiveContext();
    // 归零后更新射线
    updateTraceLine(aCtx);
    aCtx->UpdateCurrentViewer();

}

void calcRobot(QMdiArea* ws)
{



    std::size_t dof = g_rlKinematics->getDof();
    //打印各关节限位角度
    std::cout << "================ 关节限位（单位：度） ================" << std::endl;



    // 2. 逐个获取每个关节并打印限位
    for (std::size_t i = 0; i < dof; ++i)
    {
        // 用 getJoint(i) 获取第 i 个关节（rl 推荐的标准写法）
        const rl::mdl::Joint* joint = g_rlKinematics->getJoint(i);

        // joint->max / min 是 rl::math::Vector（单自由度关节取第0个元素）
        // 内部存储是弧度，必须转成角度
        double max_deg = joint->max(0) * rl::math::RAD2DEG;
        double min_deg = joint->min(0) * rl::math::RAD2DEG;

        std::cout << "Joint " << i << ": min = " << min_deg << " °, max = " << max_deg << " °" << std::endl;
    }
    std::cout << "=====================================================" << std::endl;

    rl::math::Vector qinv(6);//存储逆解计算的关节角度

    rl::math::Vector qzero(6);//初始关节角度
    // 1. 创建 RL 向量，并从当前的 Joint_angles 获取值
    rl::math::Vector q(dof);
    for (std::size_t i = 0; i < dof; ++i)
    {
        // 关键：将您在界面或 moveJoint 中改变后的角度传给 RL
        q(i) = Joint_angles[i];
        qzero(i) = 0;//置零，作为逆解起点
    }

    //正解
    g_rlKinematics->setPosition(q);
    g_rlKinematics->forwardPosition();
    //打印坐标
    rl::math::Transform t = g_rlKinematics->getOperationalPosition(0);//获取“操作空间”中第 0 个末端执行器（TCP）相对于基座的位姿矩阵

    std::cout << "rl依据xml计算末端坐标 (X,Y,Z): "
              << t.translation().x() << ", "
              << t.translation().y() << ", "
              << t.translation().z() << std::endl;
    //获取occ末端矩阵
    // 1. 获取全局变换矩阵
    const gp_Trsf& T = g_endTrihedron->Transformation();

    // 2. 获取创建 Trihedron 时定义的那个初始点 (即 Component 里的 Location)
    gp_Pnt P_initial = g_endTrihedron->Component()->Location();

    // 3. 用矩阵作用于这个点，算出它在世界坐标系里的真实位置
    gp_Pnt P_world = P_initial.Transformed(T);

    std::cout << "OCC 真实末端坐标 (X,Y,Z): "
              << P_world.X() << ", "
              << P_world.Y() << ", "
              << P_world.Z() << std::endl;

    // 反解之前将关节角度全部设0, 必需调用forwardPosition
    //For iterative inverse, set starting point far away
    g_rlKinematics->setPosition(qzero);//迭代起点
    g_rlKinematics->forwardPosition();


    //使用前面正解的t作为反解的输入
    if (g_rlKinematics->calculateInversePosition(t, 0, 0.5))//目标位姿矩阵，末端执行器，迭代误差容限或时间限制
    {
        qinv = g_rlKinematics->getPosition();
        //刷新逆解计算后的关节角位姿显示
        MDIWindow* activeWin = qobject_cast<MDIWindow*>(ws->activeSubWindow()->widget());
        DocumentCommon* doc = activeWin->getDocument();
        Handle(AIS_InteractiveContext) aCtx = doc->getContext_AIS_InteractiveContext();

        for(int i = 0; i < 6; i++)
        {
            //  将 RL 的结果同步给关节角变量
            Joint_angles[i] = qinv(i);

            //  计算 OCC 的变换矩阵
            //  Joints[i] 定义的是局部轴，这行才有效
            gp_Trsf aRodTrsf;
            aRodTrsf.SetRotation(Joints[i], Joint_angles[i]);

            // 应用到 3D 模型零件 (Rods)
            // 假设 Rods[0] 是底座, Rods[1] 是第一轴...
            if (Rods[i+1])
            {
                Rods[i+1]->SetLocalTransformation(aRodTrsf);
            }
        }

        // 逆解算出新姿态后更新射线
        updateTraceLine(aCtx);
        // 刷新视图
        aCtx->UpdateCurrentViewer();

        g_rlKinematics->setPosition(qinv); //需确认是否需要调用
        g_rlKinematics->forwardPosition();
        rl::math::Transform tinv = g_rlKinematics->getOperationalPosition(0);//利用逆解算出矩阵





        //输出比较初始关节角q和逆解关节角qinv
        //输出比较 t.matrix() 和 tinv.matrix()
        std::cout << "================ 正逆解一致性验证 ================" << std::endl;
        // 打印关节角对比 (单位：弧度)
        std::cout << "初始目标关节角 (q):    " << q.transpose() << std::endl;
        std::cout << "逆解计算关节角 (qinv): " << qinv.transpose() << std::endl;

        // 打印末端位姿矩阵对比
        std::cout << "\n正解矩阵 (t):\n" << t.matrix() << std::endl;
        std::cout << "\n逆解矩阵 (tinv):\n" << tinv.matrix() << std::endl;


    }else
    {
        //输出信息
        printf("错误：逆解计算失败，无法到达目标位姿。\n");
    }
}

void moveJoint(QMdiArea* ws,int index,int forward)
{
    Joint_angles[index]+=forward*5*3.1415926/180;//单关节驱动，每次旋转5°
    gp_Trsf aRodTrsf;
    aRodTrsf.SetRotation(Joints[index],Joint_angles[index]);//绕指定轴旋转指定角度
    Rods[index+1]->SetLocalTransformation(aRodTrsf);//index关节0操作连杆1，设置相对父连杆的局部变换

    rl::math::Vector q = g_rlKinematics->getPosition();
    q(index) = Joint_angles[index];
    g_rlKinematics->setPosition(q);
    g_rlKinematics->forwardPosition(); // 更新内部变换矩阵

    DocumentCommon* doc = qobject_cast<MDIWindow*>(
                ws->activeSubWindow()->widget() )->getDocument();
    Handle(AIS_InteractiveContext) aCtx = doc->getContext_AIS_InteractiveContext();
    // 每次点动后更新射线
    updateTraceLine(aCtx);
    aCtx->UpdateCurrentViewer();


}





int loadRobotDynamic(Handle(AIS_InteractiveContext) aCtx)
{



    // 只清除 AIS_Shape (即拆分出来的零件和之前的机器人)，保留坐标系等
    AIS_ListOfInteractive aList;
    aCtx->DisplayedObjects(aList);
    for (AIS_ListIteratorOfListOfInteractive it(aList); it.More(); it.Next()) {
        if (it.Value()->IsKind(STANDARD_TYPE(AIS_Shape))) { // 只针对形状对象
            aCtx->Remove(it.Value(), Standard_False);
        }
    }

    // 1. 弹出文件夹选择框
    QString dirPath = QFileDialog::getExistingDirectory(nullptr, "choose files", "");
    if (dirPath.isEmpty()) return 0;

    // 2. 检查并解析 robot.xml
    QString xmlFullPath = QDir(dirPath).filePath("robot.xml");//文件名写死
    QFile file(xmlFullPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        printf("错误：缺失 robot.xml 文件！\n");
        return 0;
    }

    QDomDocument doc;
    if (!doc.setContent(&file)) {
        printf("错误：解析 robot.xml 失败！\n");
        file.close();
        return 0;
    }
    file.close();

    // 3. 初始化 RL 运动学
    if (!initRLKinematics(xmlFullPath.toLocal8Bit().data())) return 0;

    // 4. 循环加载 STP 文件 (名字写死为拆分后的ROD_1_1 到 7)
    bool bLoadSucess = true;
    for(int i = 0; i < JOINT_N + 1; i++) {
        QString stpName = QString("ROD_1_%1.stp").arg(i + 1);
        QString stpFullPath = QDir(dirPath).filePath(stpName);

        Rods[i] = loadRodModel(stpFullPath.toLocal8Bit().data());
        if(Rods[i].IsNull()) {
            printf("加载模型失败: %s\n", stpFullPath.toLocal8Bit().data());
            bLoadSucess = false;
            break;
        }
    }

    if(bLoadSucess) {
        // 5. 层级装配与显示
        for(int i = 0; i < JOINT_N; i++) {
            if (!Rods[i].IsNull() && !Rods[i+1].IsNull()) {
                Rods[i]->AddChild(Rods[i+1]);
            }
        }
        for(int i = 0; i < JOINT_N + 1; i++) {
            aCtx->SetDisplayMode(Rods[i], 1, Standard_False);
            aCtx->Display(Rods[i], Standard_False);
        }

        // 从 XML 提取尺寸参数
        auto getNum = [](QDomElement parent, QString child) {
            return parent.firstChildElement(child).text().toDouble();
        };
        auto getFixedVal = [&](QString id, QString axis) {
            QDomNodeList list = doc.elementsByTagName("fixed");
            for(int i=0; i<list.count(); i++){
                QDomElement e = list.at(i).toElement();
                if(e.attribute("id") == id) return getNum(e.firstChildElement("translation"), axis);
            }
            return 0.0;
        };
        auto getJointDir = [&](int idx) {
            QDomNodeList list = doc.elementsByTagName("revolute");
            if(idx >= list.count()) return gp_Dir(0,0,1);
            QDomElement a = list.at(idx).toElement().firstChildElement("axis");
            double x = getNum(a, "x"), y = getNum(a, "y"), z = getNum(a, "z");
            return (x==0 && y==0 && z==0) ? gp_Dir(0,0,1) : gp_Dir(x,y,z);
        };

        // 获取用户在 XML 中定义的参数，后续可更改为全局变量贯穿用户输入——文件编写——xml载入中
        double R  = getFixedVal("fixed2", "x") * 1000;
        double H  = getFixedVal("fixed2", "z") * 1000;
        double L2 = getFixedVal("fixed3", "z") * 1000;
        double S  = getFixedVal("fixed4", "z") * 1000;
        double L3 = getFixedVal("fixed5", "x") * 1000;
        double D  = getFixedVal("fixed6", "y") * 1000;
        double W;//W的值随姿态不同获取位置不同，最好定义成全局变量

        // 自动识别姿态并打印
        double tx7 = getFixedVal("fixed7", "x");
        double tz7 = getFixedVal("fixed7", "z");
        bool isPose1 = (qAbs(tx7) > 0.0001);

        printf("\n--- XML 姿态识别结果 ---\n");
        if(isPose1){
            printf(">> 检测到 [姿态一：手腕平直], W = %.3f (X轴偏移)\n", tx7);
            W = tx7 * 1000;}
        else {
            printf(">> 检测到 [姿态二：手腕弯折], W = %.3f (Z轴偏移)\n", tz7);
            W = tz7 * 1000;}
        // 6. 计算旋转轴 (单位：mm)
        Joints[0] = gp_Ax1(gp_Pnt(0, 0, 0), getJointDir(0));
        Joints[1] = gp_Ax1(gp_Pnt(R, 0, H), getJointDir(1));
        Joints[2] = gp_Ax1(gp_Pnt(R, 0, H + L2), getJointDir(2));
        Joints[3] = gp_Ax1(gp_Pnt(R, 0, H + L2 + S), getJointDir(3));
        Joints[4] = gp_Ax1(gp_Pnt(R + L3, 0, H + L2 + S), getJointDir(4));

        // 根据姿态二要求：Joint 5 坐标为 [R+L3, D, 0]
        if(isPose1) Joints[5] = gp_Ax1(gp_Pnt(0, D, H + L2 + S), getJointDir(5));
        else        Joints[5] = gp_Ax1(gp_Pnt(R + L3, D, 0), getJointDir(5));

        // 7. 计算末端 TCP (pntE)
        gp_Pnt pntE;
        gp_Dir dirNE, dirVxE(1, 0, 0);
        if(isPose1) {
            pntE.SetCoord(R + L3 + W, D, H + L2 + S);
            dirNE.SetCoord(0, 0, 1);
        } else {
            pntE.SetCoord(R + L3, D, H + L2 + S + W);
            dirNE.SetCoord(0, 0, 1);
        }

        Handle(AIS_Trihedron) Trihedron = createEndTrihedron(pntE, dirNE, dirVxE);//（原点，z轴，x轴）
        if (!Rods[6].IsNull()) {
            Rods[6]->AddChild(Trihedron);
        }
        aCtx->Display(Trihedron, Standard_False);
    }
    // 首次加载模型时生成射线
    updateTraceLine(aCtx);
    aCtx->UpdateCurrentViewer();
    return bLoadSucess ? JOINT_N : 0;
}



Handle(AIS_Trihedron)  createEndTrihedron(const gp_Pnt &P, const gp_Dir &N, const gp_Dir &Vx)
{
    printf("createEndTrihedron  1\n");
    Handle(Geom_Axis2Placement)	anAxisPlacement = new Geom_Axis2Placement(gp_Ax2(P, N, Vx));
    Handle(AIS_Trihedron) trihedron = new AIS_Trihedron(anAxisPlacement);

    trihedron->SetDatumDisplayMode(Prs3d_DM_Shaded );
    trihedron->SetArrowColor(Prs3d_DatumParts_XAxis,Quantity_NOC_RED);
    trihedron->SetArrowColor(Prs3d_DP_YAxis ,Quantity_NOC_GREEN);
    trihedron->SetArrowColor(Prs3d_DatumParts_ZAxis,Quantity_NOC_BLUE);

    trihedron->SetColor(Quantity_NOC_BLUE);
    trihedron->SetXAxisColor(Quantity_NOC_RED);
    trihedron->SetYAxisColor(Quantity_NOC_GREEN);

    g_endTrihedron = trihedron;

    //trihedron->SetArrowColor(Prs3d_DatumParts_XAxis,"a");
    //trihedron->SetArrowColor(Prs3d_DatumParts_YAxis ,"S");
    //trihedron->SetArrowColor(Prs3d_DatumParts_ZAxis,"A");

    return trihedron;
}

