import sys
sys.path.append('/simply_ws/src/cgn_pytorch')
from cgn_pytorch import from_pretrained, to_onnx

cgn, _, _ = from_pretrained()
to_onnx(cgn)