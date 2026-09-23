# lib/booking.rb
module AirFlight
  class Booking
    attr_reader :theater, :manifest

    def initialize(theater)
      @theater = theater
      @manifest = []
    end

    def issue_ticket(passenger_name, seat_id)
      seat = @theater.seats[seat_id]
      return puts("❌ Seat #{seat_id} is unavailable.") unless seat
      return puts("❌ Seat #{seat_id} is already taken.") if seat[:occupant]

      passenger = {
        name: passenger_name,
        seat: seat_id,
        boarding_pass: generate_boarding_pass(passenger_name, seat_id),
        issued_at: Time.now
      }

      @theater.book_seat(seat_id, passenger)
      @manifest << passenger

      puts "🎫 Ticket issued: #{passenger_name} → Seat #{seat_id}"
      puts "   Boarding pass: #{passenger[:boarding_pass]}"
      passenger
    end

    def seat_map
      puts "\n   ┌───── SCREEN ─────┐"
      puts "   │   #{' ' * 12}   │"
      @theater.seats.values.group_by { |s| s[:row] }.each do |row, seats|
        line = "   │ "
        seats.sort_by { |s| s[:col] }.each do |s|
          line += s[:occupant] ? " 🟥 " : " 🟩 "
        end
        line += " │ Row #{row}"
        puts line
      end
      puts "   └──────────────────┘"
      puts "      A  B  C  D  E  F"
      puts "\n   🟩 = Available   🟥 = Occupied"
    end

    def manifest_report
      puts "\n📋 Passenger Manifest (#{@manifest.size} passengers)"
      puts "─" * 50
      @manifest.each do |p|
        puts "  #{p[:name].ljust(20)} Seat #{p[:seat]}  #{p[:boarding_pass]}"
      end
    end

    private

    def generate_boarding_pass(name, seat)
      prefix = "AIR"
      digest = Digest::SHA256.hexdigest("#{name}#{seat}#{Time.now.to_f}")[0..8].upcase
      "#{prefix}-#{seat}-#{digest}"
    end
  end
end