#!/usr/bin/env python3
"""Backward-compatible entrypoint for the main MacGyvBot node."""

from macgyvbot.nodes.macgyvbot_node import MacGyvBotNode, main

__all__ = ["MacGyvBotNode", "main"]


if __name__ == "__main__":
    main()
