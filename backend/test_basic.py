"""
Basic test to verify the pipeline components are working correctly.
"""
from src.config import load_config
from src.pipeline import get_pipeline_instance
from src.utils import setup_logging


def test_basic_functionality():
    """
    Test basic functionality of the pipeline components.
    """
    print("Testing basic functionality...")

    # Setup logging
    logger = setup_logging("INFO")

    try:
        # Load config (this will fail if .env is not properly set up)
        config = load_config()
        print("✓ Configuration loaded successfully")

        # Create pipeline instance
        pipeline = get_pipeline_instance(config)
        print("✓ Pipeline instance created successfully")

        # Validate components
        validation_result = pipeline.validate_pipeline()
        if validation_result:
            print("✓ Pipeline validation passed")
        else:
            print("✗ Pipeline validation failed")
            return False

        # Test with a simple local operation (not requiring network)
        test_text = "This is a test sentence for chunking validation."
        chunks = pipeline.chunker.chunk_text(test_text)
        if chunks and len(chunks) > 0:
            print(f"✓ Chunking test passed: created {len(chunks)} chunks")
        else:
            print("✗ Chunking test failed")
            return False

        print("All basic functionality tests passed!")
        return True

    except Exception as e:
        print(f"✗ Error during basic functionality test: {e}")
        return False


if __name__ == "__main__":
    success = test_basic_functionality()
    if success:
        print("\n🎉 Basic functionality verification successful!")
    else:
        print("\n❌ Basic functionality verification failed!")
        exit(1)