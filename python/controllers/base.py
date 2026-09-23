from abc import ABC, abstractmethod
import numpy as np


class Controller(ABC):
    """Abstract base class for all controllers."""

    @abstractmethod
    def compute(self, state: np.ndarray) -> float:
        """
        Compute control force given the current state.

        Args:
            state: (8,) array [x, xd, θ1, θ1d, θ2, θ2d, θ3, θ3d]

        Returns:
            F: scalar cart force [N]
        """

    def reset(self):
        """Reset any internal controller state (e.g. integrators)."""
