__all__ = ["angle_conversion_factor"]


def angle_conversion_factor(from_: str, to: str) -> float:
    """Unit conversion.

    More general implementation may be realized using astropy.units, but it's ~1000
    times slower than this thus can be a bottleneck in this time critical
    calculation.

    """
    equivalents = {"deg": 1, "arcmin": 60, "arcsec": 3600}
    try:
        return equivalents[to] / equivalents[from_]
    except KeyError:
        raise ValueError(
            "Units other than than 'deg', 'arcmin' or 'arcsec' are not supported."
        )
