"""
State management module for the Book Embeddings Ingestion Pipeline.

This module handles tracking the processing state of URLs to enable resume functionality.
"""
import json
import os
from typing import Dict, List, Optional
from .models import ProcessingState, ProcessingStatus
import logging
from datetime import datetime


class StateManager:
    """
    Class to manage the processing state of URLs for resume capability.
    """

    def __init__(self, config, state_file: str = "processing_state.json"):
        """
        Initialize the StateManager.

        Args:
            config: Configuration object
            state_file: Path to the file where state is persisted
        """
        self.config = config
        self.state_file = state_file
        self.logger = logging.getLogger(__name__)
        self._state: Dict[str, ProcessingState] = {}

        # Load existing state if file exists
        self.load_state()

    def load_state(self):
        """
        Load the processing state from the state file.
        """
        if os.path.exists(self.state_file):
            try:
                with open(self.state_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    for url, state_data in data.items():
                        processing_state = ProcessingState(
                            url=state_data['url'],
                            status=ProcessingStatus(state_data['status']),
                            retry_count=state_data.get('retry_count', 0),
                            error_message=state_data.get('error_message')
                        )
                        # Parse datetime if it exists
                        last_processed_str = state_data.get('last_processed')
                        if last_processed_str:
                            processing_state.last_processed = datetime.fromisoformat(last_processed_str)
                        self._state[url] = processing_state
                self.logger.info(f"Loaded state for {len(self._state)} URLs from {self.state_file}")
            except Exception as e:
                self.logger.error(f"Failed to load state from {self.state_file}: {str(e)}")
                self._state = {}  # Start with empty state if loading fails
        else:
            self.logger.info(f"State file {self.state_file} does not exist, starting with empty state")

    def save_state(self):
        """
        Save the current processing state to the state file.
        """
        try:
            data = {}
            for url, state in self._state.items():
                data[url] = {
                    'url': state.url,
                    'status': state.status.value,
                    'last_processed': state.last_processed.isoformat() if state.last_processed else None,
                    'retry_count': state.retry_count,
                    'error_message': state.error_message
                }

            with open(self.state_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            self.logger.info(f"Saved state for {len(self._state)} URLs to {self.state_file}")
        except Exception as e:
            self.logger.error(f"Failed to save state to {self.state_file}: {str(e)}")

    def get_state(self, url: str) -> Optional[ProcessingState]:
        """
        Get the processing state for a URL.

        Args:
            url: The URL to get state for

        Returns:
            ProcessingState if found, None otherwise
        """
        return self._state.get(url)

    def set_state(self, processing_state: ProcessingState):
        """
        Set the processing state for a URL.

        Args:
            processing_state: The ProcessingState object to save
        """
        self._state[processing_state.url] = processing_state
        self.save_state()

    def mark_processing(self, url: str):
        """
        Mark a URL as currently being processed.

        Args:
            url: The URL to mark as processing
        """
        state = self._state.get(url)
        if not state:
            state = ProcessingState(url=url)
        state.mark_processing()
        self._state[url] = state
        self.save_state()

    def mark_completed(self, url: str):
        """
        Mark a URL as successfully processed.

        Args:
            url: The URL to mark as completed
        """
        state = self._state.get(url)
        if not state:
            state = ProcessingState(url=url)
        state.mark_completed()
        self._state[url] = state
        self.save_state()

    def mark_failed(self, url: str, error_message: Optional[str] = None):
        """
        Mark a URL as failed processing.

        Args:
            url: The URL to mark as failed
            error_message: Optional error message
        """
        state = self._state.get(url)
        if not state:
            state = ProcessingState(url=url)
        state.mark_failed(error_message)
        self._state[url] = state
        self.save_state()

    def reset_state(self, url: str):
        """
        Reset the processing state for a URL to pending.

        Args:
            url: The URL to reset
        """
        state = self._state.get(url)
        if not state:
            state = ProcessingState(url=url)
        state.reset()
        self._state[url] = state
        self.save_state()

    def get_urls_by_status(self, status: ProcessingStatus) -> List[str]:
        """
        Get all URLs with a specific processing status.

        Args:
            status: The status to filter by

        Returns:
            List of URLs with the specified status
        """
        return [url for url, state in self._state.items() if state.status == status]

    def get_pending_urls(self) -> List[str]:
        """
        Get all URLs with pending status.

        Returns:
            List of pending URLs
        """
        return self.get_urls_by_status(ProcessingStatus.PENDING)

    def get_completed_urls(self) -> List[str]:
        """
        Get all URLs with completed status.

        Returns:
            List of completed URLs
        """
        return self.get_urls_by_status(ProcessingStatus.COMPLETED)

    def get_failed_urls(self) -> List[str]:
        """
        Get all URLs with failed status.

        Returns:
            List of failed URLs
        """
        return self.get_urls_by_status(ProcessingStatus.FAILED)

    def get_processing_urls(self) -> List[str]:
        """
        Get all URLs with processing status.

        Returns:
            List of processing URLs
        """
        return self.get_urls_by_status(ProcessingStatus.PROCESSING)

    def get_remaining_urls(self, all_urls: List[str]) -> List[str]:
        """
        Get URLs that still need to be processed.

        Args:
            all_urls: List of all URLs that should be processed

        Returns:
            List of URLs that haven't been completed yet
        """
        remaining = []
        for url in all_urls:
            state = self.get_state(url)
            if not state or state.status != ProcessingStatus.COMPLETED:
                remaining.append(url)
        return remaining

    def clear_state(self):
        """
        Clear all stored state.
        """
        self._state = {}
        # Also delete the state file
        if os.path.exists(self.state_file):
            os.remove(self.state_file)
        self.logger.info("Cleared all processing state")

    def get_statistics(self) -> Dict[str, int]:
        """
        Get statistics about the processing state.

        Returns:
            Dictionary with counts for each status
        """
        stats = {
            ProcessingStatus.PENDING.value: 0,
            ProcessingStatus.PROCESSING.value: 0,
            ProcessingStatus.COMPLETED.value: 0,
            ProcessingStatus.FAILED.value: 0
        }

        for state in self._state.values():
            stats[state.status.value] += 1

        return stats

    def print_statistics(self):
        """
        Print human-readable statistics about the processing state.
        """
        stats = self.get_statistics()
        total = sum(stats.values())

        print(f"\nProcessing Statistics:")
        print(f"  Total URLs tracked: {total}")
        print(f"  Pending: {stats['pending']}")
        print(f"  Processing: {stats['processing']}")
        print(f"  Completed: {stats['completed']}")
        print(f"  Failed: {stats['failed']}")
        print()


def get_state_manager_instance(config) -> StateManager:
    """
    Factory function to create a StateManager instance.

    Args:
        config: Configuration object

    Returns:
        StateManager: Configured StateManager instance
    """
    return StateManager(config)


if __name__ == "__main__":
    # Example usage
    from .config import Config
    config = Config(cohere_api_key="dummy")

    # Create state manager
    state_manager = get_state_manager_instance(config)

    # Example of using state manager
    test_urls = [
        "https://example1.com",
        "https://example2.com",
        "https://example3.com"
    ]

    # Mark first URL as processing
    state_manager.mark_processing(test_urls[0])
    print(f"Marked {test_urls[0]} as processing")

    # Mark second URL as completed
    state_manager.mark_completed(test_urls[1])
    print(f"Marked {test_urls[1]} as completed")

    # Mark third URL as failed
    state_manager.mark_failed(test_urls[2], "Timeout error")
    print(f"Marked {test_urls[2]} as failed")

    # Print statistics
    state_manager.print_statistics()

    # Get completed URLs
    completed = state_manager.get_completed_urls()
    print(f"Completed URLs: {completed}")