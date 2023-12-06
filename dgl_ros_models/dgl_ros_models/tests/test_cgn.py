import pytest
import cgn.cgn_sanity_check as cgn_check
import numpy as np
def test_cgn():
    pointcloud = np.random.rand(100, 3)
    assert len(cgn_check.check_inference(pointcloud)) == 3