#!/usr/bin/env python3
"""
Probe script to inspect the pyhula library API.
This script attempts to import pyhula and inspect its UserApi class.
"""

import sys
import inspect
import traceback

def print_section(title):
    """Print a section header."""
    print("\n" + "=" * 60)
    print(f"  {title}")
    print("=" * 60)

def safe_get_signature(obj, name):
    """Safely get the signature of a callable object."""
    try:
        if callable(obj):
            sig = inspect.signature(obj)
            return str(sig)
    except (ValueError, TypeError):
        pass
    return None

def main():
    print_section("PYHULA API PROBE")
    
    # Step 0: Installation status
    print("\n[0] Installation Status")
    print("Attempting to import pyhula...")
    
    # Step 1: Try to import pyhula
    print("\n[1] Attempting to import pyhula...")
    try:
        import pyhula
        print(f"✓ Successfully imported pyhula")
        print(f"  Location: {pyhula.__file__}")
    except ImportError as e:
        print(f"✗ Failed to import pyhula: {e}")
        print("\n" + "="*60)
        print("INSTALLATION ISSUE DETECTED")
        print("="*60)
        print("\nThe pyhula package has a build error in its source distribution.")
        print("Error: './src/pyhula/pypack/fylo/commandprocessor.py' doesn't match any files")
        print("\nThis appears to be a packaging issue with pyhula 1.1.4 and 1.1.3.")
        print("\nPossible solutions:")
        print("  1. Contact the package maintainer about the missing file")
        print("  2. Check if there's a pre-built wheel for your platform")
        print("  3. Try installing from a different source or version")
        print("  4. Manually fix the package source and install from local directory")
        print("\nBased on code analysis, pyhula.UserApi() should provide methods like:")
        print("  - connect()")
        print("  - single_fly_takeoff()")
        print("  - single_fly_turnleft(angle)")
        print("  - single_fly_forward(distance)")
        print("  - single_fly_left(distance)")
        print("  - single_fly_touchdown()")
        print("\nAnd potentially:")
        print("  - UAV_GetCoordinate()")
        print("  - UAV_GetYaw()")
        print("  - Plane_getBarrier()")
        print("  - UAV_GetFlyState()")
        sys.exit(1)
    
    # Step 2: Print module info
    print_section("Module Information")
    print(f"Module file: {pyhula.__file__}")
    
    # Check for version
    if hasattr(pyhula, '__version__'):
        print(f"Version: {pyhula.__version__}")
    elif hasattr(pyhula, 'get_version'):
        try:
            version = pyhula.get_version()
            print(f"Version (from get_version()): {version}")
        except Exception as e:
            print(f"Could not get version: {e}")
    
    # Step 3: Find UserApi class
    print_section("UserApi Class")
    UserApi = getattr(pyhula, 'UserApi', None)
    if UserApi is None:
        print("✗ UserApi class not found in pyhula module")
        print("\nAvailable attributes in pyhula:")
        for name in dir(pyhula):
            if not name.startswith('_'):
                attr = getattr(pyhula, name)
                print(f"  - {name}: {type(attr).__name__}")
        sys.exit(1)
    
    print(f"✓ Found UserApi class: {UserApi}")
    print(f"  Type: {type(UserApi)}")
    
    # Step 4: List all methods of UserApi
    print_section("UserApi Methods")
    api_methods = []
    for name in dir(UserApi):
        if not name.startswith('_'):
            attr = getattr(UserApi, name)
            if callable(attr):
                api_methods.append((name, attr))
    
    if not api_methods:
        print("No public methods found in UserApi")
    else:
        print(f"Found {len(api_methods)} public methods:\n")
        for name, method in api_methods:
            sig = safe_get_signature(method, name)
            if sig:
                print(f"  • {name}{sig}")
            else:
                print(f"  • {name}()")
    
    # Step 5: Try to instantiate UserApi
    print_section("UserApi Instantiation")
    try:
        api = UserApi()
        print("✓ Successfully instantiated UserApi()")
    except Exception as e:
        print(f"✗ Failed to instantiate UserApi: {e}")
        print(f"\nTraceback:")
        traceback.print_exc()
        sys.exit(1)
    
    # Step 6: List instance methods
    print_section("UserApi Instance Methods")
    instance_methods = []
    for name in dir(api):
        if not name.startswith('_'):
            attr = getattr(api, name)
            if callable(attr):
                instance_methods.append((name, attr))
    
    if instance_methods:
        print(f"Found {len(instance_methods)} instance methods:\n")
        for name, method in instance_methods:
            sig = safe_get_signature(method, name)
            if sig:
                print(f"  • {name}{sig}")
            else:
                print(f"  • {name}()")
    
    # Step 7: Try to call specific methods
    print_section("Testing Specific Methods")
    
    methods_to_test = [
        'UAV_GetCoordinate',
        'UAV_GetYaw',
        'Plane_getBarrier',
        'UAV_GetFlyState',
    ]
    
    for method_name in methods_to_test:
        print(f"\n[{method_name}]")
        if hasattr(api, method_name):
            method = getattr(api, method_name)
            sig = safe_get_signature(method, method_name)
            if sig:
                print(f"  Signature: {method_name}{sig}")
            
            # Try to call it
            try:
                result = method()
                print(f"  Return type: {type(result).__name__}")
                print(f"  Return value: {result}")
            except Exception as e:
                print(f"  ✗ Error calling {method_name}: {e}")
                print(f"    Exception type: {type(e).__name__}")
        else:
            print(f"  ✗ Method not found")
    
    print_section("Probe Complete")
    print("\n✓ All checks completed successfully!")

if __name__ == "__main__":
    main()
