# lib/engine.rb
require 'colorize'
require 'tty-prompt'
require 'tty-box'

module AirClone
  class Engine
    def initialize
      @state = GameState.new
      @prompt = TTY::Prompt.new
      @current_scene = nil
      @running = true
    end

    def load_scenes(scenes)
      @scenes = scenes
    end

    def start
      clear_screen
      show_title
      main_menu
    end

    private

    def main_menu
      choice = @prompt.select("Menu principal") do |menu|
        menu.choice "Nouvelle partie", :new
        menu.choice "Charger une route", :load if @state.any_route_unlocked?
        menu.choice "Quitter", :quit
      end

      case choice
      when :new  then start_new_game
      when :load then load_route_menu
      when :quit then @running = false
      end
    end

    def start_new_game
      @state.reset!
      # Commence toujours par la route DREAM
      enter_scene("dream_opening")
    end

    def load_route_menu
      routes = @state.unlocked_routes.map { |r| [r.to_s.upcase, r] }
      selected = @prompt.select("Quelle route ?", routes.to_h)
      enter_scene("#{selected}_opening")
    end

    def enter_scene(name)
      @current_scene = @scenes[name]
      raise "Scène inconnue: #{name}" unless @current_scene

      clear_screen
      show_scene_header(@current_scene)
      play_scene(@current_scene)
    end

    def play_scene(scene)
      scene.lines.each do |line|
        case line[:type]
        when :dialogue
          show_dialogue(line[:character], line[:text])
        when :narration
          show_narration(line[:text])
        end
        wait_for_input
      end

      if scene.choices.any?
        handle_choices(scene)
      elsif scene.next_scene
        enter_scene(scene.next_scene)
      else
        scene_end
      end
    end

    def handle_choices(scene)
      options = scene.choices.map { |c| c[:text] }
      selected = @prompt.select("Que fais-tu ?", options)

      choice = scene.choices.find { |c| c[:text] == selected }
      @state.set_flag(choice[:flag], choice[:value]) if choice[:flag]

      enter_scene(choice[:goto])
    end

    def show_dialogue(character, text)
      name = Character.name_of(character)
      color = Character.color_of(character)

      box = TTY::Box.frame(
        width: 70,
        padding: 1,
        title: { top_left: " #{name} ", bottom_right: " #{name} " }
      ) do
        text.colorize(color)
      end

      puts box
    end

    def show_narration(text)
      puts "\n  #{text.italic.light_black}"
    end

    def show_scene_header(scene)
      return unless scene.background || scene.bgm

      info = []
      info << "🎨 #{scene.background}" if scene.background
      info << "🎵 #{scene.bgm}"       if scene.bgm
      puts info.join("  |  ").light_black
      puts "─" * 70
    end

    def wait_for_input
      print "\n  [Entrée] "
      $stdin.gets
    end

    def scene_end
      puts "\n  ★ Fin de scène ★".yellow
      @state.save!
      main_menu
    end

    def show_title
      title = <<~TITLE

        ╔══════════════════════════════════════════╗
        ║                                          ║
        ║            A I R   C L O N E             ║
        ║                                          ║
        ║        ~ Un voyage au-delà du ciel ~     ║
        ║                                          ║
        ╚══════════════════════════════════════════╝

      TITLE
      puts title.cyan
    end

    def clear_screen
      system('clear') || system('cls')
    end
  end
end