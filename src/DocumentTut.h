#ifndef DOCUMENTTUT_H
#define DOCUMENTTUT_H

#include "DocumentCommon.h"
#include <memory>
#include <Geom_BSplineSurface.hxx>

class SurfConstruction;
class ApplicationCommon;
class AIS_Shape;
class DL_RobotContext;

class DocumentTut : public DocumentCommon
{
	Q_OBJECT
public:
   DocumentTut( const int, ApplicationCommonWindow* );
	~DocumentTut();

	int                           onMakeBottle();
	DL_RobotContext*              getRobot();

private:
	std::unique_ptr<DL_RobotContext> myRobotContext;
};


#include <AIS_InteractiveObject.hxx>
#include <Geom_Curve.hxx>

//! AIS interactive Object for Geom_Curve
class AdaptorCurve_AIS : public AIS_InteractiveObject
{
  DEFINE_STANDARD_RTTI_INLINE(AdaptorCurve_AIS, AIS_InteractiveObject)
public:
  AdaptorCurve_AIS (const Handle(Geom_Curve)& theCurve
  					,Quantity_NameOfColor color=Quantity_NOC_RED
  					,Aspect_TypeOfLine ltype=Aspect_TOL_SOLID
  					,float width=1.0);
private:

  //! Return TRUE for supported display modes (modes 0 and 1 are supported).
  virtual Standard_Boolean AcceptDisplayMode (const Standard_Integer theMode) const Standard_OVERRIDE { return theMode == 0 || theMode == 1; }

  //! Compute presentation.
  Standard_EXPORT virtual void Compute (const Handle(PrsMgr_PresentationManager)& thePrsMgr,
                                        const Handle(Prs3d_Presentation)& thePrs,
                                        const Standard_Integer theMode) Standard_OVERRIDE;

  //! Compute selection (not implemented).
  virtual void ComputeSelection (const Handle(SelectMgr_Selection)&,
                                 const Standard_Integer) Standard_OVERRIDE {}

private:
  Handle(Geom_Curve) myCurve;
  Quantity_NameOfColor myColor;
  Aspect_TypeOfLine myLineType;
  float myWidth;
  Handle(Prs3d_LineAspect) myLineAspect;
 
};

#endif
