#!/usr/bin/env python3

import argparse

import rerun as rr
import depthai as dai
import cv2
import numpy as np

from typing import List

import matplotlib.pyplot as plt


class RerunImageLogger:
  def __init__(self, name: str):
    self.name = name

  def log_frame(self, name, timestamp, frame) -> None:
    rr.set_time_seconds(self.name, timestamp)
    rr.log(f"{self.name}/{name}", rr.Image(frame, color_model="bgr"))


class AnnotationNode(dai.node.HostNode):
  def __init__(self) -> None:
    super().__init__()
    self.input_detections = self.createInput()
    self.input_frames = self.createInput()
    self.output_frames = self.createOutput(
      possibleDatatypes=[dai.Node.DatatypeHierarchy(dai.DatatypeEnum.ImgFrame, True)]
    )
    self.output_detections = self.createOutput(
      possibleDatatypes=[dai.Node.DatatypeHierarchy(dai.DatatypeEnum.ImgAnnotations, True)]
    )
    self.labels = []

  def build(
    self,
    input_detections: dai.Node.Output,
    frames: dai.Node.Output,
    labels: List[str],
  ) -> "AnnotationNode":
    self.labels = labels
    self.link_args(input_detections, frames)
    return self

  def process(self, detections_message: dai.Buffer, frame: dai.Buffer) -> None:
    assert isinstance(detections_message, dai.SpatialImgDetections)
    assert isinstance(frame, dai.ImgFrame)

    detections_list: List[dai.SpatialImgDetection] = detections_message.detections

    frame_copy = frame.getCvFrame()
    for _, detection in enumerate(detections_list):
      xmin, ymin, xmax, ymax = (
        detection.xmin,
        detection.ymin,
        detection.xmax,
        detection.ymax,
      )

      frame_copy = cv2.rectangle(
        frame_copy,
        (int(xmin * frame_copy.shape[1]), int(ymin * frame_copy.shape[0])),
        (int(xmax * frame_copy.shape[1]), int(ymax * frame_copy.shape[0])),
        color=(0, 0, 255),  # BGR format - red color
        thickness=2,
      )

      text = f"{self.labels[detection.label]} {int(detection.confidence * 100)}%"
      text += f"\nx: {detection.spatialCoordinates.x:.2f}mm"
      text += f"\ny: {detection.spatialCoordinates.y:.2f}mm"
      text += f"\nz: {detection.spatialCoordinates.z:.2f}mm"

      frame_copy = cv2.putText(
        frame_copy,
        text,
        (int((xmin + 0.01) * frame_copy.shape[1]), int((ymin + 0.2) * frame_copy.shape[0])),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 0, 0),  # BGR format - blue color
        1,
      )

    frame.setCvFrame(frame_copy, dai.ImgFrame.Type.BGR888i)

    self.output_frames.send(frame)
    self.output_detections.send(detections_message)


def main(args):
  device = dai.Device()
  platform = device.getPlatformAsString()
  print(f"Using device: {platform}")

  assert platform == "RVC2", "This script is only supported on RVC2"

  # detection model
  model_description = dai.NNModelDescription(model="luxonis/yolov6-nano:r2-coco-512x288", platform=platform)

  nn_archive = dai.NNArchive(dai.getModelFromZoo(model_description))
  fps_limit = args.fps

  with dai.Pipeline(device) as pipeline:
    print("Creating pipeline...")

    # Check if the device has color, left and right cameras
    available_cameras = device.getConnectedCameras()
    if len(available_cameras) < 3:
      raise ValueError("Device must have 3 cameras (color, left and right) in order to run this experiment.")

    # camera input
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

    left_cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    right_cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo = pipeline.create(dai.node.StereoDepth).build(
      left=left_cam.requestOutput(nn_archive.getInputSize(), fps=fps_limit),
      right=right_cam.requestOutput(nn_archive.getInputSize(), fps=fps_limit),
      presetMode=dai.node.StereoDepth.PresetMode.HIGH_DETAIL,
    )
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(*nn_archive.getInputSize())
    stereo.setLeftRightCheck(True)
    stereo.setRectification(True)

    nn = pipeline.create(dai.node.SpatialDetectionNetwork).build(
      input=cam,
      stereo=stereo,
      nnArchive=nn_archive,
      fps=float(fps_limit),
    )
    nn.setNNArchive(nn_archive, numShaves=7)
    nn.setBoundingBoxScaleFactor(0.7)

    label_map = nn.getClasses()

    # annotation
    annotation_node = pipeline.create(AnnotationNode).build(
      input_detections=nn.out,
      frames=nn.passthrough,
      labels=nn_archive.getConfig().model.heads[0].metadata.classes,
    )

    detections_queue = annotation_node.output_detections.createOutputQueue()
    camera_queue = annotation_node.output_frames.createOutputQueue()

    rerun_logger = RerunImageLogger("camera")

    pipeline.build()
    print("Starting pipeline...")
    rr.init("simple_robot_example", spawn=False)
    pipeline.start()

    fig, ax = plt.subplots()
    (com_plot,) = ax.plot([], [], "ro", markersize=14, label="COM")

    import time

    log_rate = 20
    dt = 1 / log_rate
    last_log_time = time.time()

    while pipeline.isRunning():
      camera_annotated = camera_queue.get()
      cv2.imshow("Camera", camera_annotated.getCvFrame())

      detections = detections_queue.get()

      x_data, z_data = [], []
      for detection in detections.detections:
        if label_map[detection.label] != "person":
          continue
        point = detection.spatialCoordinates
        x, y, z = map(lambda x: x / 1000, (point.x, point.y, point.z))
        x_data.append(x)
        z_data.append(z)
      com_plot.set_data(x_data, z_data)
      ax.set_xlabel("x (m)")
      ax.set_ylabel("z (m)")
      ax.set_xlim(-2, 2)
      ax.set_ylim(-0.1, 5)
      ax.set_aspect("equal")
      plt.pause(0.001)

      if time.time() - last_log_time > dt:
        rerun_logger.log_frame("camera", camera_annotated.getTimestamp().total_seconds(), camera_annotated.getCvFrame())
        rerun_logger.log_frame(
          "detections",
          camera_annotated.getTimestamp().total_seconds(),
          np.array(fig.canvas.copy_from_bbox(ax.bbox))[:, :, :3],
        )
        last_log_time = time.time()

      if cv2.waitKey(1) == ord("q"):
        break

    pipeline.stop()
    rr.save("recording.rrd")


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--fps", type=int, default=20)
  args = parser.parse_args()
  main(args)
