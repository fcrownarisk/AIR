# lib/aircraft.rb
module AirFlight
  class Aircraft
    attr_reader :altitude, :airspeed, :fuel, :cabin_pressure

    def initialize
      @altitude = 0.0          # meters
      @airspeed = 0.0          # m/s
      @fuel = 100.0            # percentage
      @cabin_pressure = 101.3  # kPa
      @engines_on = false
      @seatbelt_sign = false
    end

    def start_engines
      @engines_on = true
      puts "✈️  Engines started. Systems nominal."
    end

    def takeoff(target_altitude = 10_000)
      return unless @engines_on

      puts "🛫 Taking off..."
      while @altitude < target_altitude
        @altitude += 500
        @airspeed += 30
        @fuel -= 1.5
        @cabin_pressure = pressure_at(@altitude)
        sleep 0.3
        puts "   Alt: #{@altitude.round} m | Speed: #{@airspeed.round} m/s | Fuel: #{@fuel.round}%"
      end
      @seatbelt_sign = false
      puts "✅ Cruise altitude reached: #{@altitude.round} m"
    end

    def land
      puts "🛬 Descending..."
      while @altitude > 0
        @altitude -= 800
        @airspeed -= 40
        @fuel -= 1.0
        @altitude = 0 if @altitude < 0
        @airspeed = 0 if @airspeed < 0
        sleep 0.3
        puts "   Alt: #{@altitude.round} m | Speed: #{@airspeed.round} m/s"
      end
      puts "✅ Touchdown. Welcome to the ground."
    end

    def seatbelt_sign=(on)
      @seatbelt_sign = on
      status = on ? "ON" : "OFF"
      puts "🔔 Seatbelt sign: #{status}"
    end

    def turbulence(intensity = :mild)
      return unless @altitude > 0

      @seatbelt_sign = true
      case intensity
      when :mild     then puts "🌤️  Light turbulence. Please fasten your seatbelt."
      when :moderate then puts "⛈️  Moderate turbulence. Cabin service suspended."
      when :severe   then puts "🌪️  SEVERE TURBULENCE. BRACE FOR IMPACT."
      end
    end

    private

    def pressure_at(alt)
      # Simplified barometric formula
      101.3 * ((1 - (2.25577e-5 * alt)) ** 5.25588)
    end
  end
end