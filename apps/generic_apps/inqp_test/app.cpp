
#ifdef SHAWN
	#include "boilerplate_shawn.h"
#elif CONTIKI_TARGET_SKY
	#include "boilerplate_sky.h"
#endif

#include "static_data.h"
#include <util/meta.h>

class App : public AppBoilerplate {
	public:
		void init(Os::AppMainParameter& v) {
			AppBoilerplate::init(v);

			insert_tuples(rdf);

			timer_->set_timer<App, &App::load_predefined_query>(10000, this, (void*)10);
			result_radio().reg_recv_callback<App, &App::on_sink_receive>(this);
		}

		void load_predefined_query(void* x) {
			//debug_->debug("loading pre-installed query...");
			Uvoid x2 = (Uvoid)x;
			x2--;
			if(x2 == 0) {
				x2 = 10;
				query_processor().erase_query(QID);

				process(sizeof(op100), op100);
				process(sizeof(op90), op90);
				process(sizeof(op80), op80);
				process(sizeof(op70), op70);
				process(sizeof(cmd), cmd);
			}

			timer_->set_timer<App, &App::load_predefined_query>(10000, this, (void*)x2);
		}

		/**
		 * @param op { 'O', qid, oid, .... } or { 'Q', qid, ops }
		 */
		void process(int sz, block_data_t* op) {
			//ian_.handle_operator(op100, 0, sizeof(op100));
			//communicator_.on_receive_query(radio_->id(), sz, op);
			if(op[0] == 'O') {
				query_processor().handle_operator(op[1], sz - 2, op + 2);
			}
			else if(op[0] == 'Q') {
				query_processor().handle_query_info(op[1], op[2]);
			}
		}

		void on_sink_receive(ResultRadio::node_id_t from, ResultRadio::size_t size, ResultRadio::block_data_t *data) {
			ResultRadio::message_id_t msgid = wiselib::read<Os, block_data_t, ResultRadio::message_id_t>(data);

			//debug_->debug("@%lu sink recv %lu -> %lu s=%lu", (unsigned long)radio_->id(), (unsigned long)from, (unsigned long)result_radio_.id(), (unsigned long)SINK);
			
			if(from == SINK) {
				debug_->debug("sink recv from %lu", (unsigned long)from);
				wiselib::debug_buffer<Os, 16, Os::Debug>(debug_, data, size);
			}
		}

};

wiselib::WiselibApplication<Os, App> example_app;
void application_main(Os::AppMainParameter& value) {
  example_app.init(value);
}


