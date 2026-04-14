#include "DocumentTut.h"
#include "ApplicationTut.h"
#include "DL_RobotContext.h"
#include "View.h"

#include <Standard_WarningsDisable.hxx>
#include <QFileDialog>
#include <QMessageBox>
#include <QApplication>
#include <QStatusBar>

#include <Standard_WarningsRestore.hxx>

#include <TopoDS_Shape.hxx>
#include <AIS_Shape.hxx>
#include <AIS_ColoredShape.hxx>

#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>

#include <gp_Torus.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>

#include <gp_Sphere.hxx>
#include <Geom_SphericalSurface.hxx>

#include <gp_Elips.hxx>
#include <Geom_Ellipse.hxx>

#include <StdPrs_PoleCurve.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <Prs3d_LineAspect.hxx>

#include <GeomAdaptor_Curve.hxx>
#include <Prs3d_LineAspect.hxx>
#include <StdPrs_PoleCurve.hxx>
#include <StdPrs_Curve.hxx>

#include <Geom_BezierCurve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_Circle.hxx>
#include <Geom_OffsetCurve.hxx>
#include <Geom_Plane.hxx>
#include <AIS_Plane.hxx>
#include <AIS_AnimationObject.hxx>
#include <AIS_AnimationCamera.hxx>
#include <AIS_Circle.hxx>
#include <QMdiArea>
#include <QMdiSubWindow>


#include <TopoDS_Wire.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>

class TopoDS_Compound;

namespace
{
DocumentTut* documentFromSubWindow(QMdiSubWindow* theSubWindow)
{
    if (theSubWindow == NULL)
    {
        return NULL;
    }

    MDIWindow* aWindow = qobject_cast<MDIWindow*>(theSubWindow->widget());
    if (aWindow == NULL)
    {
        return NULL;
    }

    return qobject_cast<DocumentTut*>(aWindow->getDocument());
}

DocumentTut* activeDocument(QMdiArea* theWorkspace)
{
    if (theWorkspace == NULL)
    {
        return NULL;
    }

    if (DocumentTut* aDocument = documentFromSubWindow(theWorkspace->activeSubWindow()))
    {
        return aDocument;
    }

    QList<QMdiSubWindow*> aWindows = theWorkspace->subWindowList();
    for (int anIndex = 0; anIndex < aWindows.count(); ++anIndex)
    {
        DocumentTut* aDocument = documentFromSubWindow(aWindows.at(anIndex));
        if (aDocument != NULL)
        {
            theWorkspace->setActiveSubWindow(aWindows.at(anIndex));
            return aDocument;
        }
    }

    return NULL;
}
}

AdaptorCurve_AIS::AdaptorCurve_AIS (const Handle(Geom_Curve)& theCurve
                    ,Quantity_NameOfColor color
                    ,Aspect_TypeOfLine ltype
                    ,float width) : myCurve(theCurve),myColor(color),myLineType(ltype),myWidth(width)
{
    myLineAspect = new Prs3d_LineAspect(myColor, myLineType, myWidth);
    this->Attributes()->SetLineAspect(myLineAspect);
}

void AdaptorCurve_AIS::Compute (const Handle(PrsMgr_PresentationManager)&,
                                const Handle(Prs3d_Presentation)& thePrs,
                                const Standard_Integer theMode)
{
  GeomAdaptor_Curve anAdaptorCurve(myCurve);
  switch (theMode)
  {
    case 1:
    {
      Handle(Prs3d_Drawer) aPoleDrawer = new Prs3d_Drawer();
      aPoleDrawer->SetLineAspect(new Prs3d_LineAspect(Quantity_NOC_RED, Aspect_TOL_SOLID, 1.0));
      StdPrs_PoleCurve::Add(thePrs, anAdaptorCurve, aPoleDrawer);
    }
    Standard_FALLTHROUGH
    case 0:
    {
      //myDrawer->SetLineAspect(new Prs3d_LineAspect(Quantity_NOC_RED, Aspect_TOL_SOLID, 1.0));

      StdPrs_Curve::Add(thePrs, anAdaptorCurve, myDrawer);

      break;
    }
  }
}



TopoDS_Shape
MakeBottle(const Standard_Real myWidth , const Standard_Real myHeight , const Standard_Real myThickness);

TopoDS_Shape
MakeCube(const Standard_Real myWidth  , const Standard_Real myThickness);

DocumentTut::DocumentTut( const int theIndex, ApplicationCommonWindow* app )
: DocumentCommon( theIndex, app )
{
}

DocumentTut::~DocumentTut()
{
}

DL_RobotContext* DocumentTut::getRobot()
{
    Handle(AIS_InteractiveContext) aContext = getContext_AIS_InteractiveContext();
    if (!myRobotContext)
    {
        myRobotContext.reset(new DL_RobotContext(aContext));
    }
    else
    {
        myRobotContext->setContext(aContext);
    }

    return myRobotContext.get();

}

Handle(AIS_Shape)  AISBottle;


void ApplicationTut::onJointSelect()
{
    if (nMaxJoint_Count <= 0)
    {
        return;
    }

    nCurrentJoint_Index++;
    if(nCurrentJoint_Index==nMaxJoint_Count)
        nCurrentJoint_Index = 0;

    statusBar()->showMessage(QString("Current Joint: J#")+QString::number(nCurrentJoint_Index+1)+" of "+QString::number(nMaxJoint_Count));
}

void ApplicationTut::onJointForward()
{
    DocumentTut* aDocument = activeDocument(getWorkspace());
    if (aDocument != NULL)
    {
        aDocument->getRobot()->moveJoint(nCurrentJoint_Index, 1);
    }
}

void ApplicationTut::onJointBckward()
{
    DocumentTut* aDocument = activeDocument(getWorkspace());
    if (aDocument != NULL)
    {
        aDocument->getRobot()->moveJoint(nCurrentJoint_Index, -1);
    }
}


void ApplicationTut::onRobotReset()
{
    DocumentTut* aDocument = activeDocument(getWorkspace());
    if (aDocument != NULL)
    {
        aDocument->getRobot()->resetRobot();
    }
}

void ApplicationTut::onRobotCalc()
{
    DocumentTut* aDocument = activeDocument(getWorkspace());
    if (aDocument != NULL)
    {
        aDocument->getRobot()->calcRobot();
    }
}

void ApplicationTut::onRobotDisas()
{
    DocumentTut* aDocument = activeDocument(getWorkspace());
    if (aDocument != NULL)
    {
        aDocument->getRobot()->disasRobot(getWorkspace());
    }
    else
    {
        QMessageBox::warning(this, "Tips", "Please open a occ3D Window!");
    }
}

void ApplicationTut::onRobotWrite()
{
    DL_RobotContext::writeRobotXml(getWorkspace());
}




int DocumentTut::onMakeBottle()
{
    QApplication::setOverrideCursor( Qt::WaitCursor );

    int nBjoint = getRobot()->loadRobotDynamic(ApplicationCommonWindow::getWorkspace());

    fitAll();
    QApplication::restoreOverrideCursor();

    return nBjoint;
}

   /* TopoDS_Shape aBottle=MakeBottle(50,70,30);
    AISBottle = new AIS_Shape(aBottle);
    aCtx->Display(AISBottle, Standard_False);
    //AISBottleX = AISBottle;
    //TopoDS_Shape aBottle=MakeCube(50,4);
    //Handle(AIS_Shape) AISBottle=new AIS_Shape(aBottle);
    //----------------------------------------------------------------------------------------

    gp_Torus aBaseTorus(gp_Ax3(gp_Pnt(), gp_Dir(0.0, 0.0, 1.0)), 40.0, 10.0);
    Handle(Geom_ToroidalSurface) aBaseSurface = new Geom_ToroidalSurface(aBaseTorus);
    Handle(AIS_Shape) AISBottle = new AIS_Shape(BRepBuilderAPI_MakeFace(
        aBaseSurface
        , 0.0, 2.0*M_PI
        , 0.0, 1.0*M_PI
        , Precision::Confusion()).Shape());

    aCtx->SetMaterial (AISBottle, Graphic3d_NameOfMaterial_Gold, Standard_False);
    aCtx->SetDisplayMode(AISBottle, 1, Standard_False);
    aCtx->Display(AISBottle, Standard_False);

    //----------------------------------------------------------------------------------------

    gp_Sphere aSphere(gp_Ax3(), 10.0);
    Handle(Geom_SphericalSurface) aSphericalSurface = new Geom_SphericalSurface(aSphere);
    AISBottle = new AIS_Shape(BRepBuilderAPI_MakeFace(
    aSphericalSurface, 0.0, 2.0*M_PI, 0.0, 2.0*M_PI, Precision::Confusion()).Shape());
    */
    /*
    gp_Elips anElips(gp_Ax2(gp_Pnt(), gp_Dir(1.0, 0.0, 0.0)), 20.0, 10.0);
    Handle(Geom_Ellipse) aGeomEllipse = new Geom_Ellipse(anElips);
    Handle(AdaptorCurve_AIS) anAisScaledEllipce = new AdaptorCurve_AIS(aGeomEllipse);
    aCtx->Display(anAisScaledEllipce,false);


    // Define points.
  gp_Pnt aPnt1(0.0, 0.0, 0.0);
  gp_Pnt aPnt2(5.0, 5.0, 0.0);
  gp_Pnt aPnt3(10.0, 5.0, 0.0);
  gp_Pnt aPnt4(15.0, 0.0, 0.0);

  // Add points to the curve poles array.
  TColgp_Array1OfPnt aPoles(1, 4);
  aPoles.SetValue(1, aPnt1);
  aPoles.SetValue(2, aPnt2);
  aPoles.SetValue(3, aPnt3);
  aPoles.SetValue(4, aPnt4);

    // Define Bezier weights.
  TColStd_Array1OfReal aBezierWeights(1, 4);
  aBezierWeights.SetValue(1, 0.5);
  aBezierWeights.SetValue(2, 1.5);
  aBezierWeights.SetValue(3, 1.5);
  aBezierWeights.SetValue(4, 0.5);

  // Create Bezier curve.
  //Handle(Geom_BezierCurve) aBezierCurve = new Geom_BezierCurve(aPoles, aBezierWeights);
    // Define BSpline weights.
  TColStd_Array1OfReal aBSplineWeights(1, 4);
  aBSplineWeights.SetValue(1, 1.0);
  aBSplineWeights.SetValue(2, 0.5);
  aBSplineWeights.SetValue(3, 0.5);
  aBSplineWeights.SetValue(4, 1.0);

  // Define knots.
  TColStd_Array1OfReal aKnots(1, 2);
  aKnots.SetValue(1, 0.0);
  aKnots.SetValue(2, 1.0);

  // Define multiplicities.
  TColStd_Array1OfInteger aMults(1, 2);
  aMults.SetValue(1, 4);
  aMults.SetValue(2, 4);

  // Define BSpline degree and periodicity.
  Standard_Integer aDegree = 3;
  Standard_Boolean aPeriodic = Standard_False;

  Handle(Geom_BSplineCurve) aBSplineCurve = new Geom_BSplineCurve(
    aPoles, aBSplineWeights, aKnots, aMults, aDegree, aPeriodic);

   Handle(AdaptorCurve_AIS) anAisScaledEllipce2 = new AdaptorCurve_AIS(aBSplineCurve,Quantity_NOC_GREEN, Aspect_TOL_SOLID,3.0);


    gp_Trsf aRotTrsf;
    aRotTrsf.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(1.0, 0.0, 0.0)), M_PI_2);
    anAisScaledEllipce2->SetLocalTransformation(aRotTrsf);

    aCtx->Display(anAisScaledEllipce2,false);


    BRepBuilderAPI_MakeWire wireMaker;
    BRepBuilderAPI_MakeEdge edgeMaker1(aPnt1, aPnt2);
    if (edgeMaker1.IsDone()) {
        wireMaker.Add(edgeMaker1.Edge()); // 将线段添加到Wire中
    }
    BRepBuilderAPI_MakeEdge edgeMaker2(aPnt2, aPnt3);
    if (edgeMaker2.IsDone()) {
        wireMaker.Add(edgeMaker2.Edge()); // 将线段添加到Wire中
    }
    BRepBuilderAPI_MakeEdge edgeMaker3(aPnt3, aPnt4);
    if (edgeMaker3.IsDone()) {
        wireMaker.Add(edgeMaker3.Edge()); // 将线段添加到Wire中
    }
    if (wireMaker.IsDone())
    {
        TopoDS_Wire myPolyline = wireMaker.Wire();
        Handle(AIS_Shape) myPolyline2 =new AIS_Shape(myPolyline);
        aCtx->Display(myPolyline2, Standard_False);
    }

    */
    /*
    gp_Circ aCirc(gp::XOY(), 5.0);
  // Create a closed circular curve.
  Handle(Geom_Circle) aCircCurve = new Geom_Circle(aCirc);

  Standard_Real anExpandOffset = +aCirc.Radius() / 4.0;
  gp_Dir anExpandDir = gp::DZ();
  Handle(Geom_OffsetCurve) anExpandCircCurve = new Geom_OffsetCurve(
    aCircCurve, anExpandOffset, anExpandDir);

  Handle(AIS_ColoredShape) anAisCirc = new AIS_ColoredShape (BRepBuilderAPI_MakeEdge(aCircCurve).Shape());
  Handle(AIS_ColoredShape) anAisExpandCirc = new AIS_ColoredShape (BRepBuilderAPI_MakeEdge(anExpandCircCurve).Shape());
  //Handle(AIS_ColoredShape) anAisCpllapsedCirc = new AIS_ColoredShape (BRepBuilderAPI_MakeEdge(anCollapseCircCurve).Shape());
  anAisCirc->SetColor(Quantity_Color(Quantity_NOC_YELLOW));
  anAisExpandCirc->SetColor(Quantity_Color(Quantity_NOC_RED));
  //anAisCpllapsedCirc->SetColor(Quantity_Color(Quantity_NOC_GREEN));
  aCtx->Display(anAisCirc, Standard_False);
  aCtx->Display(anAisExpandCirc, Standard_False);


    gp_Pln aPln(gp::XOY());
  // Create a plane surface.
  Handle(Geom_Plane) aPlaneSurf = new Geom_Plane(aPln);

  Handle(AIS_Plane) anAisPlane1 = new AIS_Plane(aPlaneSurf);
  Handle(AIS_ColoredShape) anAisPlane = new AIS_ColoredShape(
    BRepBuilderAPI_MakeFace(aPln, 0.0, 10.0, 0.0, 10.0));

    aCtx->Display(anAisPlane1, Standard_False);
    aCtx->Display(anAisPlane, AIS_Shaded, 0, Standard_False);   */
    //aCtx->SetMaterial (AISBottle, Graphic3d_NameOfMaterial_Gold, Standard_False);
   // aCtx->SetDisplayMode(AISBottle, 1, Standard_False);
    //aCtx->Display(AISBottle, Standard_False);
    /*const Handle(AIS_InteractiveObject)& anIOAISBottle = AISBottle;
    getContext_AIS_InteractiveContext()->SetSelected(anIOAISBottle,Standard_False);
    emit selectionChanged();




    QMdiArea* ws = getWorkspace();
    DocumentCommon* doc = qobject_cast<MDIWindow*>(
        ws->activeSubWindow()->widget() )->getDocument();

    Handle(AIS_InteractiveContext) aCtx = doc->getContext_AIS_InteractiveContext();
    gp_Trsf start_trsf, end_trsf;
    gp_Ax1 ax1(gp_Pnt(10, 0, 0), gp_Vec(0, 1, 0));
    end_trsf.SetRotation(ax1, M_PI_2);
    Handle(AIS_AnimationObject) ani_object = new AIS_AnimationObject("Object", aCtx, AISBottle, start_trsf, end_trsf);
    ani_object->SetOwnDuration(5.0); // 5秒
    ani_object->StartTimer(0, 1.0, true); // 开始定时器
    while (!ani_object->IsStopped())
    {
        ani_object->UpdateTimer();
        aCtx->UpdateCurrentViewer();
    }


        //box1a.AddChild(boxshp1a)
      /*
    // Make an edge from two 3D points.
    int gridWidth = 10;

    for(int x=0;x<=10; x+=5)
    {
        gp_Pnt aPnt1(x, 0, 0.0);
        gp_Pnt aPnt2(x, gridWidth, 0.0);
        TopoDS_Edge anEdgeP12 = BRepBuilderAPI_MakeEdge(aPnt1, aPnt2);
        Handle(AIS_ColoredShape) anAisEdgeP12 = new AIS_ColoredShape(anEdgeP12);
        anAisEdgeP12->SetColor(Quantity_Color(Quantity_NOC_YELLOW));
        aCtx->Display(anAisEdgeP12, Standard_False);
    }

    for(int y=0;y<=10; y+=5)
    {
        gp_Pnt aPnt1(0, y, 0.0);
        gp_Pnt aPnt2(gridWidth, y, 0.0);
        TopoDS_Edge anEdgeP12 = BRepBuilderAPI_MakeEdge(aPnt1, aPnt2);
        Handle(AIS_ColoredShape) anAisEdgeP12 = new AIS_ColoredShape(anEdgeP12);
        anAisEdgeP12->SetColor(Quantity_Color(Quantity_NOC_RED));
        aCtx->Display(anAisEdgeP12, Standard_False);
    }

    gp_Ax2 anAxis2(gp_Pnt(0.0, 5.0, 0.0), gp_Dir(0.0, 0.0, 1.0));
    gp_Circ aCirc(anAxis2, 2.5);
    Handle(Geom_Circle) aGeomCircle = new Geom_Circle(aCirc);
    Handle(AIS_Circle) anAisCircle = new AIS_Circle(aGeomCircle);

    gp_Ax2 anAxis1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0));
    gp_Circ aCirc1(anAxis1, 1);
    Handle(Geom_Circle) aGeomCircle1 = new Geom_Circle(aCirc1);
    anAisCircle1 = new AIS_Circle(aGeomCircle1);
   */
    //gp_Vec trans(gp_Pnt(), gp_Pnt(5.0, 5.0, 0.0));
    //gp_Trsf _gp_Trsf = gp_Trsf ();
    //_gp_Trsf.SetTranslation(trans);
    //anAisCircle->SetLocalTransformation(_gp_Trsf);
    //anAisCircle1->UpdateTransformation();

    //anAisCircle->AddChild(anAisCircle1);
    //anAisCircle1->UpdateTransformation();


   //	anAisCircle->AddChildWithCurrentTransformation(anAisCircle1);

  //  gp_Vec trans(gp_Pnt(), gp_Pnt(3.0, 3.0, 0.0));
  // 	gp_Trsf _gp_Trsf = gp_Trsf ();
  // 	_gp_Trsf.SetTranslation(trans);

  ///

  //  aCtx->Display(anAisCircle, Standard_False);
 //   aCtx->Display(anAisCircle1, Standard_False);


/*	QMdiArea* ws = getWorkspace();
    MDIWindow* mw = qobject_cast<MDIWindow*>( ws->activeSubWindow()->widget() );
    Handle(V3d_View) _V3d_View = mw->getView()->getView_V3d_View();

    DocumentCommon* doc = qobject_cast<MDIWindow*>(
        ws->activeSubWindow()->widget() )->getDocument();

    Handle(AIS_InteractiveContext) aCtx = doc->getContext_AIS_InteractiveContext();

    gp_Trsf end_trsf;
    gp_Ax1 ax1(gp_Pnt(10, 0, 0), gp_Vec(0, 1, 0));
    end_trsf.SetRotation(ax1, M_PI_2);
    Handle(Graphic3d_Camera) camera_start = _V3d_View->Camera();
    Handle(Graphic3d_Camera) camera_end = new Graphic3d_Camera();
    camera_end->Copy(camera_start);
    camera_end->Transform(end_trsf);
    Handle(AIS_AnimationCamera) ani_camera = new AIS_AnimationCamera("Camera", _V3d_View);
    ani_camera->SetCameraStart(camera_start);
    ani_camera->SetCameraEnd(camera_end);

    //Handle(AIS_InteractiveContext) aCtx = doc->getContext_AIS_InteractiveContext();
    ani_camera->SetOwnDuration(5.0); // 5秒
    ani_camera->StartTimer(0, 1.0, true); // 开始定时器
    while (!ani_camera->IsStopped())
    {
        ani_camera->UpdateTimer();
        aCtx->UpdateCurrentViewer();
    }

    QMdiArea* ws = getWorkspace();
    MDIWindow* mw = qobject_cast<MDIWindow*>( ws->activeSubWindow()->widget() );
    Handle(V3d_View) _V3d_View = mw->getView()->getView_V3d_View();

    gp_Vec trans(gp_Pnt(), gp_Pnt(-5.0, 5.0, 0.0));
    gp_Trsf _gp_Trsf = gp_Trsf ();
    _gp_Trsf.SetTranslation(trans);
    anAisCircle1->SetLocalTransformation(_gp_Trsf);

    _V3d_View->Redraw();*/
