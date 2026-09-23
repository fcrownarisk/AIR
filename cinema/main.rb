# main.rb
require 'digest'
require_relative 'data/air_media'
require_relative 'lib/aircraft'
require_relative 'lib/theater'
require_relative 'lib/booking'
require_relative 'lib/inflight_service'

module AirFlight
  class FlightCinema
    def initialize
      @aircraft = Aircraft.new
      @theater = Theater.new(@aircraft)
      @booking = Booking.new(@theater)
      @service = InFlightService.new(@theater)
    end

    def run
      banner
      book_passengers
      @booking.seat_map

      @aircraft.start_engines
      @aircraft.seatbelt_sign = true
      @aircraft.takeoff(10_000)

      @theater.turn_on_screen
      @theater.set_volume(0.8)
      @theater.toggle_subtitles

      # Simulate turbulence during the flight
      @aircraft.turbulence(:mild)

      # Play all 12 episodes during the 12-hour flight
      AIR_MEDIA[:episodes].each do |ep|
        @theater.play(ep)
        sleep 0.2
      end

      @service.serve_drinks
      @service.serve_meals

      @aircraft.seatbelt_sign = true
      @aircraft.land

      @service.order_summary
      @booking.manifest_report
      debrief
    end

    private

    def banner
      puts <<~BANNER

        ╔══════════════════════════════════════════════════════════╗
        ║         ✈️  A I R F L I G H T   C I N E M A  🎬         ║
        ║                                                          ║
        ║   A movie theater aboard an aircraft                     ║
        ║   Featuring the complete AIR anime series (2005)         ║
        ║   Kyoto Animation · Key / Visual Art's                   ║
        ║                                                          ║
        ║   Bilibili Rating: #{AIR_MEDIA[:bilibili_rating]} ★  (#{AIR_MEDIA[:bilibili_views] / 10_000}万播放)   ║
        ╚══════════════════════════════════════════════════════════╝

      BANNER
    end

    def book_passengers
      passengers = [
        ["神尾観鈴 (Misuzu)", "1A"],
        ["国崎往人 (Yukito)", "1B"],
        ["霧島佳乃 (Kano)",   "2A"],
        ["遠野美凪 (Minagi)", "2B"],
        ["神尾晴子 (Haruko)", "3A"]
      ]

      passengers.each { |name, seat| @booking.issue_ticket(name, seat) }
    end

    def debrief
      puts <<~DEBRIEF

        ═══════════════════════════════════════════════════════════
          FLIGHT DEBRIEF
        ═══════════════════════════════════════════════════════════
          Aircraft .......... Boeing 787-9 (Cinema Retrofit)
          Route ............. Tokyo → Okinawa (via the Sky)
          Duration .......... 12 hours (one episode per hour)
          Passengers ........ #{@theater.passengers.size}
          Episodes Screened . #{AIR_MEDIA[:episodes].size}
          Soundtrack ........ #{AIR_MEDIA[:soundtrack].map { |s| s[:title] }.join(', ')}
          Final Status ...... Landed safely
        ═══════════════════════════════════════════════════════════
      DEBRIEF
    end
  end
end

AirFlight::FlightCinema.new.run if __FILE__ == $PROGRAM_NAME