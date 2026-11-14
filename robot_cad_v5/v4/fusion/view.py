"""Camera helper: look at a robot-frame target from a robot-frame eye (mm), save PNGs."""
import os

import adsk.core

import cadlib as C


def look(eye, target, up=(0, 0, 1), fit=True, ortho=False):
    vp = C.app().activeViewport
    cam = vp.camera
    e, t, u = C.rf(eye), C.rf(target), C.rf(up)
    cam.eye = adsk.core.Point3D.create(e[0] * C.CM, e[1] * C.CM, e[2] * C.CM)
    cam.target = adsk.core.Point3D.create(t[0] * C.CM, t[1] * C.CM, t[2] * C.CM)
    cam.upVector = adsk.core.Vector3D.create(u[0], u[1], u[2])
    cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType if ortho else adsk.core.CameraTypes.PerspectiveCameraType
    cam.isFitView = fit
    vp.camera = cam
    if fit:
        vp.fit()
    vp.refresh()


def snap(path, eye, target, w=1600, h=1200, ortho=False, up=(0, 0, 1), fit=True):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    look(eye, target, up=up, ortho=ortho, fit=fit)
    C.app().activeViewport.saveAsImageFile(path, w, h)
    return path
