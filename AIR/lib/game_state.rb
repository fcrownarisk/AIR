# lib/game_state.rb
require 'json'

module AirClone
  class GameState
    SAVE_FILE = File.expand_path('../save.json', __dir__)

    attr_reader :flags, :unlocked_routes

    def initialize
      reset!
    end

    def reset!
      @flags = {}
      @unlocked_routes = [:dream]
      @completed_routes = []
    end

    def set_flag(name, value = true)
      return if name.nil?
      @flags[name.to_sym] = value
      check_route_unlocks
    end

    def flag?(name)
      @flags[name.to_sym] == true
    end

    def complete_route!(route)
      @completed_routes << route unless @completed_routes.include?(route)
      check_route_unlocks
      save!
    end

    def any_route_unlocked?
      @unlocked_routes.length > 1
    end

    def save!
      data = {
        flags: @flags,
        unlocked_routes: @unlocked_routes,
        completed_routes: @completed_routes
      }
      File.write(SAVE_FILE, JSON.pretty_generate(data))
    end

    def load!
      return unless File.exist?(SAVE_FILE)
      data = JSON.parse(File.read(SAVE_FILE), symbolize_names: true)
      @flags = data[:flags] || {}
      @unlocked_routes = (data[:unlocked_routes] || [:dream]).map(&:to_sym)
      @completed_routes = (data[:completed_routes] || []).map(&:to_sym)
    end

    private

    def check_route_unlocks
      # SUMMER se débloque après avoir terminé les 3 routes de DREAM
      dream_complete = %i[kano misuzu minagi].all? { |r| @completed_routes.include?(r) }
      @unlocked_routes << :summer if dream_complete && !@unlocked_routes.include?(:summer)

      # AIR se débloque après SUMMER
      if @completed_routes.include?(:summer) && !@unlocked_routes.include?(:air)
        @unlocked_routes << :air
      end
    end
  end
end