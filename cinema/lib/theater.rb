# lib/theater.rb
module AirFlight
  class Theater
    attr_reader :seats, :current_screen, :passengers

    SCREEN_WIDTH = 12  # meters — large enough for a cabin cinema
    SCREEN_HEIGHT = 6

    def initialize(aircraft)
      @aircraft = aircraft
      @seats = {}
      @passengers = []
      @current_screen = nil
      @screen_on = false
      @volume = 0.7
      @subtitles_on = true
      build_seats
    end

    def build_seats(rows: 8, cols: 6)
      rows.times do |r|
        cols.times do |c|
          # 2-2-2 layout with aisles
          col_letter = ("A".."F").to_a[c]
          @seats["#{r + 1}#{col_letter}"] = {
            row: r + 1,
            col: col_letter,
            occupant: nil,
            recline: 0,
            screen_angle: angle_to_screen(r + 1, c)
          }
        end
      end
    end

    def book_seat(seat_id, passenger)
      seat = @seats[seat_id]
      return false unless seat
      return false if seat[:occupant]

      seat[:occupant] = passenger
      @passengers << passenger
      true
    end

    def available_seats
      @seats.select { |_, s| s[:occupant].nil? }.keys
    end

    def turn_on_screen
      @screen_on = true
      puts "📽️  Cabin cinema activated. Screen: #{SCREEN_WIDTH}m × #{SCREEN_HEIGHT}m"
    end

    def play(episode)
      return unless @screen_on

      @current_screen = episode
      puts "\n🎬 Now playing: AIR Episode #{episode[:num]} — #{episode[:title]}"
      puts "   Screen angle adjusted for all #{@seats.size} seats."

      # Auto-adjust seat angles during takeoff
      if @aircraft.altitude < 3000
        @seats.each_value { |s| s[:screen_angle] += 2 }
      end
    end

    def set_volume(level)
      @volume = level.clamp(0.0, 1.0)
      puts "🔊 Volume: #{(level * 100).round}%"
    end

    def toggle_subtitles
      @subtitles_on = !@subtitles_on
      puts "💬 Subtitles: #{@subtitles_on ? 'ON' : 'OFF'}"
    end

    private

    def angle_to_screen(row, col)
      # Seats closer to the screen have a steeper viewing angle
      base_angle = 90 - (row * 5)
      base_angle += (col - 2.5) * 2
      base_angle
    end
  end
end