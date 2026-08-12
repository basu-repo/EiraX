import pytest

from scripts.check_px4_links import parse_links


def test_preflight_link_parser_accepts_unique_swarm_links():
    assert parse_links(
        ["dji0=udpin:0.0.0.0:14540", "dji1=udpin:0.0.0.0:14541"]
    ) == [
        ("dji0", "udpin:0.0.0.0:14540"),
        ("dji1", "udpin:0.0.0.0:14541"),
    ]


@pytest.mark.parametrize(
    "links",
    [
        ["bad"],
        ["=udpin:0.0.0.0:14540"],
        ["dji0="],
        ["dji0=x", "dji0=y"],
        ["dji0=x", "dji1=x"],
    ],
)
def test_preflight_link_parser_rejects_ambiguous_links(links):
    with pytest.raises(Exception):
        parse_links(links)
