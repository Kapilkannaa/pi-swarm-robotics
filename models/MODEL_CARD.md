# Red round target classifier

Binary logistic regression, six image features, threshold 0.8. Inference and
training math use standard-library Python on the Pi; the laptop extracts features
with OpenCV. Model/schema: `target.json`.

Positive: saturated red approximately round token. Negatives: other-colored round
objects and red elongated rectangles. Features: red excess, saturation, brightness,
contour circularity, bounding-box occupancy and aspect ratio. A red colour mask
proposes candidates before classification.

Reproduce: `python scripts/train_target.py`. Training seed 42, 1,200 generated
labeled images. Held-out evaluation seed 901, 400 generated images: TP=200, TN=200,
FP=0, FN=0 (100%). Variations: brightness, saturation, size, blur, rotation and small
ellipticity. This is synthetic performance, not a real-room accuracy claim.

Real-camera validation: **not yet done**. Label actual scene samples with varied
lighting and confusing objects; evaluate on a separate camera session. Other round
red objects may be false positives. Perspective, clutter, reflections and shadows
are not comprehensively modeled. Not a general object detector, CNN, or robot-side TinyML.

The Pi gates discovery using model probability and the explicit virtual sensing
radius. It separately learns each robot's movement characteristics online. No API key.
