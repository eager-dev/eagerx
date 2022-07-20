import typing as t
import gym.spaces as spaces
import numpy as np
from functools import reduce
import operator as op


class Space(spaces.Space):
    """
    A (possibly unbounded) space in R^n. Specifically, a Space represents the
    Cartesian product of n closed intervals. Each interval has the form of one
    of [a, b], (-oo, b], [a, oo), or (-oo, oo).

    There are two common use cases:

    * Identical bound for each dimension::
        >>> Space(low=-1.0, high=2.0, shape=(3, 4), dtype="float32")
        Space(3, 4)

    * Independent bound for each dimension::
        >>> Space(low=np.array([-1.0, -2.0]), high=np.array([2.0, 4.0]), dtype="float32")
        Space(2,)

    """

    def __init__(
        self,
        low: t.Optional[t.Union[t.List, np.ndarray, int, float, bool, t.Tuple]] = None,
        high: t.Optional[t.Union[t.List, np.ndarray, int, float, bool, t.Tuple]] = None,
        shape: t.Optional[t.Tuple] = None,
        dtype: t.Union[np.number, str] = np.float32,
        seed: t.Optional[int] = None,
    ):
        """Constructor for initializing a space.

        :param low: Expected lower bound of messages.
        :param high: Expected upper bound of messages.
        :param shape: Expected shape of messages.
        :param dtype: Expected datatype of messages.
        :param seed: Seed that is used to draw random samples from the space.
        """
        if isinstance(low, (list, tuple)):
            low = np.array(low, dtype=dtype)
        if isinstance(high, (list, tuple)):
            high = np.array(high, dtype=dtype)

        # determine shape if it isn't provided directly
        if shape is not None:
            shape = tuple(shape)
            assert low is None or np.isscalar(low) or low.shape == shape, "low.shape doesn't match provided shape"
            assert high is None or np.isscalar(high) or high.shape == shape, "high.shape doesn't match provided shape"
        elif low is not None and not np.isscalar(low):
            shape = low.shape
            assert high is not None and np.isscalar(high) or high.shape == shape, "high.shape doesn't match low.shape"
        elif high is not None and not np.isscalar(high):
            shape = high.shape
            assert low is not None and np.isscalar(low) or low.shape == shape, "low.shape doesn't match high.shape"
        elif shape is None:
            pass
        else:
            raise ValueError("shape must be provided or inferred from the shapes of low or high")

        if np.isscalar(low):
            low = np.full(shape, low, dtype=dtype)

        if np.isscalar(high):
            high = np.full(shape, high, dtype=dtype)

        if low is not None and high is not None:
            self._space = spaces.Box(low=low, high=high, shape=shape, dtype=dtype, seed=seed)
        else:
            self._low = low
            self._high = high
            self._space = None
        super().__init__(shape=shape, dtype=dtype, seed=seed)

    def __getattr__(self, name):
        if self._space is None:
            raise AttributeError("Attribute does not exist.")
        else:
            return getattr(self._space, name)

    def contains(self, x) -> bool:
        """
        Return boolean specifying if x is a valid
        member of this space
        :param x: array to check.
        """
        if self._space is None:
            # todo: convert to ndarray? Maybe not to test that dtype is correct && we are dealing with np array?
            # if not isinstance(x, np.ndarray):
            #     x = np.asarray(x, dtype=self.dtype)

            return x.dtype == self.dtype and (self.shape is None or x.shape == self.shape)
        else:
            return x.dtype == self.dtype and self._space.contains(x)  # Check dtype separately

    def contains_space(self, space: t.Union["Space", t.Dict]) -> bool:
        """
        Return boolean specifying if space is contained
        in this space.
        Low and high of the space must exactly match (instead of lying within the bounds) to return True
        :param space: Space that is to be checked.
        """
        if isinstance(space, dict):
            space = Space.from_dict(space)

        # Check dtype
        flag = space.dtype == self.dtype

        # Check shape
        if flag and space.shape is not None and self.shape is not None:
            flag = flag and space.shape == self.shape

        # Check bounds
        if flag and space._space is not None and self._space is not None:
            flag = flag and (space._space.low == self._space.low).all()
            flag = flag and (space._space.high == self._space.high).all()

        return flag

    def sample(self) -> np.ndarray:
        """Randomly sample an element of this space. Can be
        uniform or non-uniform sampling based on boundedness of space."""
        if not self.is_fully_defined:
            raise NotImplementedError("Cannot sample from this incomplete space (i.e. low, high not provided?).")
        else:
            return self._space.sample()

    def to_dict(self) -> t.Dict:
        """Convert the space to a dict representation

        :return: Dict representation of the space.
        """
        if self._space is None:
            if self.shape is None:
                return dict(low=None, high=None, shape=None, dtype=self.dtype.name)
            else:
                return dict(low=None, high=None, shape=list(self.shape), dtype=self.dtype.name)
        else:
            low = np.nan_to_num(self.low)
            high = np.nan_to_num(self.high)
            if np.all(low == low.reshape(-1)[0]):
                low = low.reshape(-1)[0].item()
            if np.all(high == high.reshape(-1)[0]):
                high = high.reshape(-1)[0].item()
            if isinstance(low, np.ndarray):
                low = low.tolist()
            if isinstance(high, np.ndarray):
                high = high.tolist()
            return dict(low=low, high=high, shape=list(self.shape), dtype=self.dtype.name)

    @property
    def is_fully_defined(self) -> bool:
        """Check if space is fully defined (i.e. low, high, shape and dtype are all provided).
        :return: flag
        """
        return self._space is not None

    @classmethod
    def from_dict(cls, d: t.Dict) -> "Space":
        """Create a space from a dict.

        :param d: Dict containing the arguments to initialize the space
        :return: The space.
        """
        return cls(low=d["low"], high=d["high"], shape=d["shape"], dtype=d["dtype"])

    def __repr__(self):
        if self._space is None:
            if self.shape is None:
                return f"Space({self.dtype})"
            else:
                return f"Space({self.shape}, {self.dtype})"
        else:
            return f"Space({self.low}, {self.high}, {self.shape}, {self.dtype})"

    def __eq__(self, other):
        if self._space is None:
            return (
                isinstance(other, Space)
                and other._space is None
                and (self.shape == other.shape)
                and (self.dtype == other.dtype)
            )
        else:
            return (
                isinstance(other, spaces.Box)
                and (self.shape == other.shape)
                and np.allclose(self.low, other.low)
                and np.allclose(self.high, other.high)
            )


# Register a method to flatten our custom Space using standard gym functions.
@spaces.utils.flatten_space.register(Space)
def flatten_space_box(space):
    return spaces.Box(space.low.flatten(), space.high.flatten(), dtype=space.dtype)


# Register a method to unflatten our custom Space using standard gym functions.
@spaces.utils.unflatten.register(Space)
def unflatten_box(space, x):
    return np.asarray(x, dtype=space.dtype).reshape(space.shape)


@spaces.utils.flatdim.register(Space)
def flatdim_box_multibinary(space):
    return reduce(op.mul, space.shape, 1)


@spaces.utils.flatten.register(Space)
def flatten_box_multibinary(space, x):
    return np.asarray(x, dtype=space.dtype).flatten()
