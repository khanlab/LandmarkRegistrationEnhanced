import vtk, qt, ctk, slicer
from . import RegistrationPlugin


#########################################################
#
#
comment = """

  RegistrationPlugin is a superclass for code that plugs into the
  slicer LandmarkRegistration module.

  These classes are Abstract.

# TODO :
"""
#
#########################################################



#
# RegistrationPlugin
#

class AffineAndThinPlatePlugin(RegistrationPlugin):
  """ Plugin for thin plat spline using vtk
  """

  #
  # generic settings that can (should) be overridden by the subclass
  #

  # displayed for the user to select the registration
  name = "Affine And ThinPlate Registration"
  tooltip = "Uses landmarks to define Affine and nonlinear warp transform"

  # can be true or false
  # - True: landmarks are displayed and managed by LandmarkRegistration
  # - False: landmarks are hidden
  usesLandmarks = True

  # can be any non-negative number
  # - widget will be disabled until landmarks are defined
  landmarksNeededToEnable = 1

  # used for reloading - every concrete class should include this
  sourceFile = __file__

  def __init__(self,parent=None):
    super(AffineAndThinPlatePlugin,self).__init__(parent)
    self.landmarkTransform = None
    self.thinPlateTransform = None

  def create(self,registationState):
    """Make the plugin-specific user interface"""
    super(AffineAndThinPlatePlugin,self).create(registationState)
    #
    # Thin Plate Spline Registration Pane
    #
    self.thinPlateCollapsibleButton = ctk.ctkCollapsibleButton()
    self.thinPlateCollapsibleButton.text = "Affine And Thin Plate Spline Registration"
    affineAndThinPlateFormLayout = qt.QFormLayout()
    self.thinPlateCollapsibleButton.setLayout(affineAndThinPlateFormLayout)
    self.widgets.append(self.thinPlateCollapsibleButton)

	
    self.linearMode = "Rigid"
    
    self.registrationModeBox = qt.QGroupBox("Registration Mode")
    self.registrationModeBox.setLayout(qt.QFormLayout())

    buttonLayout = qt.QVBoxLayout()
    self.linearModeButtons = {}
    self.linearModes = ("Rigid", "Similarity", "Affine")
    for mode in self.linearModes:
       self.linearModeButtons[mode] = qt.QRadioButton()
       self.linearModeButtons[mode].text = mode
       self.linearModeButtons[mode].setToolTip( "Run the registration in %s mode." % mode )
       self.linearModeButtons[mode].connect('clicked()', lambda m=mode : self.onLinearTransform(m))
       self.registrationModeBox.layout().addWidget(self.linearModeButtons[mode])

    self.linearModeButtons[self.linearMode].checked = True
    affineAndThinPlateFormLayout.addWidget(self.registrationModeBox)

    self.hotUpdateButton = qt.QCheckBox("Hot Update")
    affineAndThinPlateFormLayout.addWidget(self.hotUpdateButton)
    self.widgets.append(self.hotUpdateButton)

    self.exportGridButton = qt.QPushButton("Export to Grid Transform")
    self.exportGridButton.toolTip = "To save this transform or use it in other Slicer modules you can export the current Thin Plate transform to a Grid Transform."
    affineAndThinPlateFormLayout.addWidget(self.exportGridButton)
    self.exportGridButton.connect("clicked()",self.onExportGrid)
    self.widgets.append(self.exportGridButton)

    self.parent.layout().addWidget(self.thinPlateCollapsibleButton)

  def destroy(self):
    """Clean up"""
    super(AffineAndThinPlatePlugin,self).destroy()

  def onExportGrid(self):
    """Converts the current thin plate transform to a grid"""
    state = self.registationState()

    # since the transform is ras-to-ras, we find the extreme points
    # in ras space of the fixed (target) volume and fix the unoriented
    # box around it.  Sample the grid transform at the resolution of
    # the fixed volume, which may be a bit overkill but it should aways
    # work without too much loss.
    rasBounds = [0,]*6
    state.fixed.GetRASBounds(rasBounds)
    from math import floor, ceil
    origin = map(int,map(floor,rasBounds[::2]))
    maxes = map(int,map(ceil,rasBounds[1::2]))
    boundSize = [m - o for m,o in zip(maxes,origin) ]
    spacing = state.fixed.GetSpacing()
    samples = [ceil(b / s) for b,s in zip(boundSize,spacing)]
    extent = [0,]*6
    extent[::2] = [0,]*3
    extent[1::2] = samples
    extent = map(int,extent)

    toGrid = vtk.vtkTransformToGrid()
    toGrid.SetGridOrigin(origin)
    toGrid.SetGridSpacing(state.fixed.GetSpacing())
    toGrid.SetGridExtent(extent)
    toGrid.SetInput(state.transform.GetTransformFromParent()) # same in VTKv 5 & 6
    toGrid.Update()

    gridTransform = vtk.vtkGridTransform()
    if vtk.VTK_MAJOR_VERSION < 6:
      gridTransform.SetDisplacementGrid(toGrid.GetOutput()) # different in VTKv 5 & 6
    else:
      gridTransform.SetDisplacementGridData(toGrid.GetOutput())
    gridNode = slicer.vtkMRMLGridTransformNode()
    gridNode.SetAndObserveTransformFromParent(gridTransform)
    gridNode.SetName(state.transform.GetName()+"-grid")
    slicer.mrmlScene.AddNode(gridNode)

  def onLinearTransform(self,mode):
    self.linearMode = mode
    self.onAffineAndThinPlateApply()

  def onLandmarkMoved(self,state):
    """Called when the user changes a landmark"""
    if self.hotUpdateButton.checked:
      self.onAffineAndThinPlateApply()

  def onLandmarkEndMoving(self,state):
    """Called when the user changes a landmark"""
    self.onAffineAndThinPlateApply()

  def onAffineAndThinPlateApply(self):
    """Call this whenever thin plate needs to be calculated"""
    state = self.registationState()

    if not state.fixedFiducials or not state.movingFiducials:
      return

    if state.fixed and state.moving and state.transformed:
      landmarks = state.logic.landmarksForVolumes((state.fixed, state.moving))
      self.performAffineAndThinPlateRegistration(state, landmarks)
    
  def performAffineAndThinPlateRegistration(self, state, landmarks):
    """Perform Affine registration first, use the transformed result as the input of the thin plate transform"""
    
    # if state.transformed:
     # if state.transformed.GetTransformNodeID() != state.transform.GetID():
        # state.transformed.SetAndObserveTransformNodeID(state.transform.GetID())


    volumeNodes = (state.fixed, state.moving)
    fiducialNodes = (state.fixedFiducials,state.movingFiducials)
    points = state.logic.vtkPointsForVolumes( volumeNodes, fiducialNodes )
 
    #yingli debug
    #print 'self.linearMode',self.linearMode

    if not self.landmarkTransform:
        self.landmarkTransform = vtk.vtkLandmarkTransform()

    if self.linearMode == 'Rigid':
      self.landmarkTransform.SetModeToRigidBody()
    if self.linearMode == 'Similarity':
      self.landmarkTransform.SetModeToSimilarity()
    if self.linearMode == 'Affine':
      self.landmarkTransform.SetModeToAffine()
    if state.fixedFiducials.GetNumberOfFiducials() < 3:
      self.landmarkTransform.SetModeToRigidBody()
   
    self.landmarkTransform.SetSourceLandmarks(points[state.moving])
    self.landmarkTransform.SetTargetLandmarks(points[state.fixed])
    self.landmarkTransform.Update()

    #transform moving landmarks
    affine_transformed_moving_points = vtk.vtkPoints()
    self.landmarkTransform.TransformPoints(points[state.moving],affine_transformed_moving_points)

    #yingli debug
    #print self.landmarkTransform.GetMatrix()
    #print 'old moving', points[state.moving].GetPoint(0)
    #print 'new moving', affine_transformed_moving_points.GetPoint(0)

    # do thin plate, use affine transformed result as the input
    # since this is a resample transform, source is the fixed (resampling target) space
    # and moving is the target space
    if not self.thinPlateTransform:
      self.thinPlateTransform = vtk.vtkThinPlateSplineTransform()
    self.thinPlateTransform.SetBasisToR() # for 3D transform
    self.thinPlateTransform.SetSourceLandmarks(affine_transformed_moving_points)
    self.thinPlateTransform.SetTargetLandmarks(points[state.fixed])
    self.thinPlateTransform.Update()

    if points[state.moving].GetNumberOfPoints() != points[state.fixed].GetNumberOfPoints():
      raise hell

    #add nesting transfrom: order matters!
    transformSelector = slicer.qMRMLNodeComboBox()
    transformSelector.nodeTypes = ( ("vtkMRMLTransformNode"), "" )
    transformSelector.selectNodeUponCreation = True
    transformSelector.addEnabled = True
    transformSelector.removeEnabled = True
    transformSelector.noneEnabled = True
    transformSelector.showHidden = False
    transformSelector.showChildNodeTypes = False
    transformSelector.setMRMLScene( slicer.mrmlScene )
    transformSelector.setToolTip( "The transform for linear registration" )
    transformSelector.enabled = False

    # concatenate transfroms: method 1: add nesting transfrom: order matters!
    # note: this method will keep each transform(not ideal)
    
    # landmarkTransformNode = slicer.util.getNode('Affine-Transform')
    # if not landmarkTransformNode:
        # landmarkTransformNode=transformSelector.addNode()
        # landmarkTransformNode.SetName('Affine-Transform')
        # state.transform.SetAndObserveTransformNodeID(landmarkTransformNode.GetID())

    # landmarkTransformNode.ApplyTransform(self.landmarkTransform)

    # thinPlateTransformNode = slicer.util.getNode('ThinPlate-Transform')
    # if not thinPlateTransformNode:
        # thinPlateTransformNode=transformSelector.addNode()
        # thinPlateTransformNode.SetName('ThinPlate-Transform')
        # landmarkTransformNode.SetAndObserveTransformNodeID(thinPlateTransformNode.GetID())

    # # thinPlateTransformNode.ApplyTransform(self.thinPlateTransform)
    
    #state.transform.SetAndObserveTransformToParent(self.landmarkTransform)
    # #state.transform.SetAndObserveTransformToParent(self.thinPlateTransform)
    # stateTransformNodeNode = slicer.util.getNode('Transform')
    # stateTransformNodeNode.SetAndObserveTransformToParent(self.thinPlateTransform)


    #test vtk concatenate
    #self.landmarkTransform.Concatenate(self.landmarkTransform)
    #state.transform.SetAndObserveTransformToParent(self.thinPlateTransform)

    #concatenate transfroms: method 2: use vtkGeneralTransform to concatenate transfroms.
    transform = vtk.vtkGeneralTransform()
    transform.Concatenate(self.thinPlateTransform)
    transform.Concatenate(self.landmarkTransform)
    state.transform.SetAndObserveTransformToParent(transform)

# Add this plugin to the dictionary of available registrations.
# Since this module may be discovered before the Editor itself,
# create the list if it doesn't already exist.
try:
  slicer.modules.registrationPlugins
except AttributeError:
  slicer.modules.registrationPlugins = {}
slicer.modules.registrationPlugins['AffineAndThinPlate'] = AffineAndThinPlatePlugin
