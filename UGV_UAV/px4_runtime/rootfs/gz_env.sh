#!/usr/bin/env bash
# -----------------------------------------------------------------------
# Gazebo Environment Configuration
# -----------------------------------------------------------------------
# GZ_SIM_RESOURCE_PATH: Where Gazebo looks for models and worlds
# GZ_SIM_SYSTEM_PLUGIN_PATH: Where Gazebo looks for plugin libraries
# GZ_SIM_SERVER_CONFIG_PATH: Custom Gazebo server configuration file
#
# See Gazebo docs
# https://gazebosim.org/api/sim/8/resources.html
# https://gazebosim.org/api/sim/8/server_config.html
# -----------------------------------------------------------------------

PX4_RUNTIME=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
export PX4_GZ_MODELS="$PX4_RUNTIME/models"
export PX4_GZ_WORLDS="$PX4_RUNTIME/worlds"
export PX4_GZ_PLUGINS="$PX4_RUNTIME/plugins"

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$PX4_GZ_MODELS:$PX4_GZ_WORLDS
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:$PX4_GZ_PLUGINS

# macOS: dyld does not search /opt/homebrew/lib by default. Many
# gobject-introspection typelibs reference libs by bare basename,
# which fails to resolve unless we add Homebrew's lib dir to the
# fallback search path. Prepend so it beats any system-placed libs,
# and preserve any existing value the user has set.
if [ "$(uname)" = "Darwin" ]; then
	HOMEBREW_PREFIX=${HOMEBREW_PREFIX:-$(brew --prefix 2>/dev/null)}
	if [ -n "$HOMEBREW_PREFIX" ]; then
		export DYLD_FALLBACK_LIBRARY_PATH="$HOMEBREW_PREFIX/lib:${DYLD_FALLBACK_LIBRARY_PATH}"
	fi
fi
