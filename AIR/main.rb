# main.rb
$LOAD_PATH.unshift File.expand_path('lib', __dir__)

require_relative 'lib/engine'
require_relative 'lib/dsl'
require_relative 'lib/game_state'
require_relative 'data/characters'
require_relative 'data/dream_route'

module AirClone
  # Enregistre tous les scénarios
  SCENES = {
    **DREAM_ROUTE.scenes,
    # **SUMMER_ROUTE.scenes,
    # **AIR_ROUTE.scenes
  }

  engine = Engine.new
  engine.load_scenes(SCENES)
  engine.start
end