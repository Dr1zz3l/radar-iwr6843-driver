#!/usr/bin/env python3
"""Test script to verify rosbag_loader module functionality."""

import sys
from pathlib import Path

# Add module to path
analysis_dir = Path(__file__).parent
sys.path.insert(0, str(analysis_dir))

def test_imports():
    """Test that all imports work."""
    print("Testing imports...")
    try:
        from rosbag_loader import (
            load_bag_topics,
            inspect_bag_topics,
            MocapPose,
            MocapAccel,
            AgirosState,
            AgirosOdometry,
            IMUData,
            RadarPointCloud,
            RadarVelocity,
            BagData,
        )
        print("✓ All imports successful")
        return True
    except ImportError as e:
        print(f"✗ Import error: {e}")
        return False


def test_dataclass_creation():
    """Test that dataclasses can be instantiated."""
    print("\nTesting dataclass creation...")
    try:
        import numpy as np
        from rosbag_loader import MocapPose, AgirosState, IMUData
        
        # Create test instances
        pose = MocapPose(
            timestamp=0.0,
            position=np.array([1.0, 2.0, 3.0]),
            orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        )
        
        state = AgirosState(
            timestamp=0.0,
            position=np.array([1.0, 2.0, 3.0]),
            velocity=np.array([0.1, 0.2, 0.3]),
            orientation=np.array([0.0, 0.0, 0.0, 1.0]),
            angular_velocity=np.array([0.01, 0.02, 0.03]),
        )
        
        imu = IMUData(
            timestamp=0.0,
            linear_acceleration=np.array([0.0, 0.0, -9.81]),
            angular_velocity=np.array([0.0, 0.0, 0.0]),
        )
        
        # Test to_dict() method
        pose_dict = pose.to_dict()
        assert 'x' in pose_dict and 'y' in pose_dict and 'z' in pose_dict
        
        state_dict = state.to_dict()
        assert 'vx' in state_dict and 'vy' in state_dict and 'vz' in state_dict
        
        imu_dict = imu.to_dict()
        assert 'ax' in imu_dict and 'ay' in imu_dict and 'az' in imu_dict
        
        print("✓ Dataclass creation and to_dict() methods work")
        return True
    except Exception as e:
        print(f"✗ Dataclass error: {e}")
        return False


def test_bag_existence():
    """Check if test bag files exist."""
    print("\nChecking rosbag files...")
    rosbags_dir = Path("/workspace/rosbags")
    
    if not rosbags_dir.exists():
        print(f"✗ Directory not found: {rosbags_dir}")
        return False
    
    bags = list(rosbags_dir.glob("*.bag"))
    if bags:
        print(f"✓ Found {len(bags)} rosbag file(s):")
        for bag in bags:
            size_mb = bag.stat().st_size / (1024 * 1024)
            print(f"  - {bag.name} ({size_mb:.1f} MB)")
        return True
    else:
        print("✗ No rosbag files found")
        return False


def test_inspect_bag():
    """Test bag inspection functionality."""
    print("\nTesting bag inspection...")
    try:
        from rosbag_loader import inspect_bag_topics
        from pathlib import Path
        
        bags = list(Path("/workspace/rosbags").glob("*.bag"))
        if not bags:
            print("⊘ Skipping (no bags found)")
            return None
        
        bag_path = str(bags[0])
        print(f"Inspecting: {Path(bag_path).name}")
        
        topics_info = inspect_bag_topics(bag_path)
        
        if topics_info:
            print(f"✓ Found {len(topics_info)} topics")
            return True
        else:
            print("✗ No topics found")
            return False
            
    except Exception as e:
        print(f"✗ Inspection error: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all tests."""
    print("=" * 60)
    print("RosBag Loader Module - Test Suite")
    print("=" * 60)
    
    results = []
    
    # Run tests
    results.append(("Imports", test_imports()))
    results.append(("Dataclasses", test_dataclass_creation()))
    results.append(("Rosbag Files", test_bag_existence()))
    results.append(("Bag Inspection", test_inspect_bag()))
    
    # Summary
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    
    passed = sum(1 for _, r in results if r is True)
    failed = sum(1 for _, r in results if r is False)
    skipped = sum(1 for _, r in results if r is None)
    
    for name, result in results:
        if result is True:
            status = "✓ PASS"
        elif result is False:
            status = "✗ FAIL"
        else:
            status = "⊘ SKIP"
        print(f"  {status}: {name}")
    
    print(f"\nTotal: {passed} passed, {failed} failed, {skipped} skipped")
    
    if failed > 0:
        print("\n⚠ Some tests failed. Check the output above.")
        return 1
    else:
        print("\n✓ All tests passed!")
        return 0


if __name__ == "__main__":
    sys.exit(main())
