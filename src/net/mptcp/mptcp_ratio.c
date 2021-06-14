/* MPTCP Scheduler module selector. Highly inspired by tcp_cong.c */

#include <linux/module.h>
#include <net/mptcp.h>

//#ifdef MPTCP_SCHED_PROBE
//void sprobe_init(struct mptcp_sched_probe *sprobe)
//{
//    sprobe->id = 0;
//    sprobe->sk = NULL;
//    sprobe->selector_reject = false;
//    sprobe->found_unused_reject = false;
//    sprobe->def_unavailable = false;
//    sprobe->temp_unavailable = false;
//    sprobe->srtt_reject = false;
//    sprobe->selected = false;
//    sprobe->split=0;
//    sprobe->skblen=0;
//}
//
///* This exists only for kretprobe to hook on to and read sprobe */
//noinline struct mptcp_sched_probe* mptcp_sched_probe_log_hook(struct mptcp_sched_probe* sprobe, bool selected, unsigned long sched_probe_id, struct sock *sk) {
//    sprobe->selected = selected;
//    sprobe->id = sched_probe_id;
//    sprobe->sk = sk;
//
//    return sprobe;
//}
//EXPORT_SYMBOL_GPL(mptcp_sched_probe_log_hook);
//#endif

static unsigned char num_segments __read_mostly = 100;
module_param(num_segments, byte, 0644);
MODULE_PARM_DESC(num_segments, "The number of consecutive segments that are part of a burst");

static bool cwnd_limited __read_mostly = 0;
module_param(cwnd_limited, bool, 0644);
MODULE_PARM_DESC(cwnd_limited, "if set to 1, the scheduler tries to fill the congestion-window on all subflows");

/*static unsigned char num_segments_flow_one __read_mostly = 77;
module_param(num_segments_flow_one, byte, 0644);
MODULE_PARM_DESC(num_segments_flow_one, "The number of segments (out of num_segments) to be assigned to the first flow");
*/

struct ratio_sched_priv {
	unsigned char quota;
};

static struct ratio_sched_priv *ratio_sched_get_priv(const struct tcp_sock *tp)
{
	return (struct ratio_sched_priv *)&tp->mptcp->mptcp_sched[0];
}

/* If the sub-socket sk available to send the skb? */
static bool mptcp_ratio_is_available(const struct sock *sk, const struct sk_buff *skb,
				  bool zero_wnd_test, bool cwnd_test)
{
	const struct tcp_sock *tp = tcp_sk(sk);
	unsigned int space, in_flight;

	/* Set of states for which we are allowed to send data */
	if (!mptcp_sk_can_send(sk))
		return false;

	/* We do not send data on this subflow unless it is
	 * fully established, i.e. the 4th ack has been received.
	 */
	if (tp->mptcp->pre_established)
		return false;

	if (tp->pf)
		return false;

	if (inet_csk(sk)->icsk_ca_state == TCP_CA_Loss) {
		/* If SACK is disabled, and we got a loss, TCP does not exit
		 * the loss-state until something above high_seq has been acked.
		 * (see tcp_try_undo_recovery)
		 *
		 * high_seq is the snd_nxt at the moment of the RTO. As soon
		 * as we have an RTO, we won't push data on the subflow.
		 * Thus, snd_una can never go beyond high_seq.
		 */
		if (!tcp_is_reno(tp))
			return false;
		else if (tp->snd_una != tp->high_seq)
			return false;
	}

	if (!tp->mptcp->fully_established) {
		/* Make sure that we send in-order data */
		if (skb && tp->mptcp->second_packet &&
		    tp->mptcp->last_end_data_seq != TCP_SKB_CB(skb)->seq)
			return false;
	}

	if (!cwnd_test)
		goto zero_wnd_test;

	in_flight = tcp_packets_in_flight(tp);
	/* Not even a single spot in the cwnd */
	if (in_flight >= tp->snd_cwnd)
		return false;

	/* Now, check if what is queued in the subflow's send-queue
	 * already fills the cwnd.
	 */
	space = (tp->snd_cwnd - in_flight) * tp->mss_cache;

	if (tp->write_seq - tp->snd_nxt > space)
		return false;

zero_wnd_test:
	if (zero_wnd_test && !before(tp->write_seq, tcp_wnd_end(tp)))
		return false;

	return true;
}

/* Are we not allowed to reinject this skb on tp? */
static int mptcp_ratio_dont_reinject_skb(const struct tcp_sock *tp, const struct sk_buff *skb)
{
	/* If the skb has already been enqueued in this sk, try to find
	 * another one.
	 */
	return skb &&
		/* Has the skb already been enqueued into this subsocket? */
		mptcp_pi_to_flag(tp->mptcp->path_index) & TCP_SKB_CB(skb)->path_mask;
}

/* We just look for any subflow that is available */
static struct sock *ratio_get_available_subflow(struct sock *meta_sk,
					     struct sk_buff *skb,
					     bool zero_wnd_test)
{
	const struct mptcp_cb *mpcb = tcp_sk(meta_sk)->mpcb;
	struct sock *sk, *bestsk = NULL, *backupsk = NULL;

//#ifdef MPTCP_SCHED_PROBE
//    struct sock *sk_it;
//    struct mptcp_sched_probe sprobe;
//    unsigned long sched_probe_id;
//
//    sprobe_init(&sprobe);
//    get_random_bytes(&sched_probe_id, sizeof(sched_probe_id));
//#endif    
	
    /* Answer data_fin on same subflow!!! */
	if (meta_sk->sk_shutdown & RCV_SHUTDOWN &&
	    skb && mptcp_is_data_fin(skb)) {
		mptcp_for_each_sk(mpcb, sk) {
//#ifdef MPTCP_SCHED_PROBE
//			if (tcp_sk(sk)->mptcp->path_index == mpcb->dfin_path_index &&
//			    mptcp_ratio_is_available(sk, skb, zero_wnd_test, true)) {
//                if (sk) mptcp_sched_probe_log_hook(&sprobe, true, sched_probe_id, sk);
//                return sk;
//            }
//#else
//            if (tcp_sk(sk)->mptcp->path_index == mpcb->dfin_path_index &&
//                mptcp_ratio_is_available(sk, skb, zero_wnd_test, true))
//                return sk;
//#endif
		}
	}

	/* First, find the best subflow */
	mptcp_for_each_sk(mpcb, sk) {
		struct tcp_sock *tp = tcp_sk(sk);

		if (!mptcp_ratio_is_available(sk, skb, zero_wnd_test, true))
			continue;

		if (mptcp_ratio_dont_reinject_skb(tp, skb)) {
			backupsk = sk;
			continue;
		}

		bestsk = sk;
	}

	if (bestsk) {
		sk = bestsk;
	} else if (backupsk) {
		/* It has been sent on all subflows once - let's give it a
		 * chance again by restarting its pathmask.
		 */
		if (skb)
			TCP_SKB_CB(skb)->path_mask = 0;
		sk = backupsk;
	}
//#ifdef MPTCP_SCHED_PROBE
//    mptcp_for_each_sk(mpcb, sk_it) {
//        if (sk && sk_it == sk) mptcp_sched_probe_log_hook(&sprobe, true, sched_probe_id, sk);
//        else mptcp_sched_probe_log_hook(&sprobe, false, sched_probe_id, sk);
//    }
//#endif
	return sk;
}

/* Returns the next segment to be sent from the mptcp meta-queue.
 * (chooses the reinject queue if any segment is waiting in it, otherwise,
 * chooses the normal write queue).
 * Sets *@reinject to 1 if the returned segment comes from the
 * reinject queue. Sets it to 0 if it is the regular send-head of the meta-sk,
 * and sets it to -1 if it is a meta-level retransmission to optimize the
 * receive-buffer.
 */
static struct sk_buff *__mptcp_ratio_next_segment(const struct sock *meta_sk, int *reinject)
{
	const struct mptcp_cb *mpcb = tcp_sk(meta_sk)->mpcb;
	struct sk_buff *skb = NULL;

	*reinject = 0;

	/* If we are in fallback-mode, just take from the meta-send-queue */
	if (mpcb->infinite_mapping_snd || mpcb->send_infinite_mapping)
		return tcp_send_head(meta_sk);

	skb = skb_peek(&mpcb->reinject_queue);

	if (skb)
		*reinject = 1;
	else
		skb = tcp_send_head(meta_sk);
	return skb;
}

//shivanga

#define tcp_probe_copy_fl_to_si4(inet, si4, mem)        \
    do {                            \
        si4.sin_family = AF_INET;           \
        si4.sin_port = inet->inet_##mem##port;      \
        si4.sin_addr.s_addr = inet->inet_##mem##addr;   \
    } while (0)                     \


static struct sk_buff *mptcp_ratio_next_segment(struct sock *meta_sk,
					     int *reinject,
					     struct sock **subsk,
					     unsigned int *limit)
{
	const struct mptcp_cb *mpcb = tcp_sk(meta_sk)->mpcb;
	struct sock *sk_it, *choose_sk = NULL;
	struct sk_buff *skb = __mptcp_ratio_next_segment(meta_sk, reinject);
	unsigned char split = num_segments;
	unsigned char iter = 0, full_subs = 0, counter = 0;
//#ifdef MPTCP_SCHED_PROBE
//    struct mptcp_sched_probe sprobe;
//    unsigned long sched_probe_id;
//    
//    get_random_bytes(&sched_probe_id, sizeof(sched_probe_id));
//    sprobe_init(&sprobe);    
//#endif

	/* As we set it, we have to reset it as well. */
	*limit = 0;

	if (!skb)
		return NULL;

	if (*reinject) {
		*subsk = ratio_get_available_subflow(meta_sk, skb, false);
		if (!*subsk)
			return NULL;

		return skb;
	}


retry:
	/* First, we look for a subflow who is currently being used */
	mptcp_for_each_sk(mpcb, sk_it) {
		struct tcp_sock *tp_it = tcp_sk(sk_it);
		struct ratio_sched_priv *rsp = ratio_sched_get_priv(tp_it);
		const struct inet_sock *inet = inet_sk(sk_it);
	        union {
	            struct sockaddr     raw;
	            struct sockaddr_in  v4;
        	    struct sockaddr_in6 v6;
	        } dst;
		
		counter++;
		tcp_probe_copy_fl_to_si4(inet, dst.v4, d);
	        //printk("shivanga: %pISpc\n",&dst);		
                if (!mptcp_ratio_is_available(sk_it, skb, false, cwnd_limited))
			continue;

		iter++;
        
        	if (counter % 2) {
		    if (sysctl_num_segments_flow_one == 0) {
		        full_subs++;
		        continue;
		    }
        	    /* Is this subflow currently being used? */
        	    if (rsp->quota > 0 && rsp->quota < sysctl_num_segments_flow_one) {
        	        split = sysctl_num_segments_flow_one - rsp->quota;
        	        choose_sk = sk_it;
        	        goto found;
        	    }

        	    /* Or, it's totally unused */
        	    if (!rsp->quota) {
        	        split = sysctl_num_segments_flow_one;
        	        choose_sk = sk_it;
        	    }

        	    /* Or, it must then be fully used  */
        	    if (rsp->quota >= sysctl_num_segments_flow_one)
        	        full_subs++;    
        	} 
        	else {
	                    if (num_segments - sysctl_num_segments_flow_one == 0) {
                		full_subs++;
		                continue;
		            }
			    /* Is this subflow currently being used? */
			    if (rsp->quota > 0 && rsp->quota < (num_segments - sysctl_num_segments_flow_one)) {
				    split = (num_segments - sysctl_num_segments_flow_one) - rsp->quota;
				    choose_sk = sk_it;
				    goto found;
			    }

			    /* Or, it's totally unused */
			    if (!rsp->quota) {
				    split = num_segments - sysctl_num_segments_flow_one;
				    choose_sk = sk_it;
			    }

			    /* Or, it must then be fully used  */
			    if (rsp->quota >= (num_segments - sysctl_num_segments_flow_one))
				    full_subs++;
        	}
	}

	/* All considered subflows have a full quota, and we considered at
	 * least one.
	 */
	if (iter && iter == full_subs) {
		/* So, we restart this round by setting quota to 0 and retry
		 * to find a subflow.
		 */
		mptcp_for_each_sk(mpcb, sk_it) {
			struct tcp_sock *tp_it = tcp_sk(sk_it);
			struct ratio_sched_priv *rsp = ratio_sched_get_priv(tp_it);

			if (!mptcp_ratio_is_available(sk_it, skb, false, cwnd_limited))
				continue;

			rsp->quota = 0;
		}

		goto retry;
	}

found:
	if (choose_sk) {
		unsigned int mss_now;
		struct tcp_sock *choose_tp = tcp_sk(choose_sk);
		struct ratio_sched_priv *rsp = ratio_sched_get_priv(choose_tp);
                const struct inet_sock *inet = inet_sk(choose_sk);
                union {
	            struct sockaddr     raw;
	            struct sockaddr_in  v4;
	            struct sockaddr_in6 v6;
        	} dst;

		if (!mptcp_ratio_is_available(choose_sk, skb, false, true))
			return NULL;

	        tcp_probe_copy_fl_to_si4(inet, dst.v4, d);
        	//printk("shivanga: %pISpc\n",&dst);
		*subsk = choose_sk;
		mss_now = tcp_current_mss(*subsk);
		*limit = split * mss_now;

		if (skb->len > mss_now)
			rsp->quota += DIV_ROUND_UP(skb->len, mss_now);
		else
			rsp->quota++;

//#ifdef MPTCP_SCHED_PROBE
//        iter = 0;
//        mptcp_for_each_sk(mpcb, sk_it) {
//            sprobe_init(&sprobe);
//            iter++;
//
//            if (!mptcp_ratio_is_available(sk_it, skb, false, cwnd_limited)) sprobe.temp_unavailable = true;
//            
//            if (choose_sk == sk_it) {
//                mptcp_sched_probe_log_hook(&sprobe, true, sched_probe_id, sk_it);
//            }
//            else mptcp_sched_probe_log_hook(&sprobe, false, sched_probe_id, sk_it);
//        }
//        mptcp_for_each_sk(mpcb, sk_it) {
//            sprobe_init(&sprobe);
//            sched_probe_id = ULONG_MAX;
//            if (choose_sk == sk_it) {
//                sprobe.split = split;
//                sprobe.skblen = DIV_ROUND_UP(skb->len, mss_now);
//                mptcp_sched_probe_log_hook(&sprobe, true, sched_probe_id, sk_it);
//                break;
//            }
//        }
//#endif
		return skb;
	}
//#ifdef MPTCP_SCHED_PROBE
//    iter = 0;
//    mptcp_for_each_sk(mpcb, sk_it) {
//        sprobe_init(&sprobe);
//        iter++;
//        
//        if (!mptcp_ratio_is_available(sk_it, skb, false, cwnd_limited)) sprobe.temp_unavailable = true;
//
//        if (choose_sk == sk_it) {
//            mptcp_sched_probe_log_hook(&sprobe, true, sched_probe_id, sk_it);
//        }
//        else mptcp_sched_probe_log_hook(&sprobe, false, sched_probe_id, sk_it);
//    }
//#endif
	return NULL;
}

static struct mptcp_sched_ops mptcp_sched_ratio = {
	.get_subflow = ratio_get_available_subflow,
	.next_segment = mptcp_ratio_next_segment,
	.name = "ratio",
	.owner = THIS_MODULE,
};

static int __init ratio_register(void)
{
	BUILD_BUG_ON(sizeof(struct ratio_sched_priv) > MPTCP_SCHED_SIZE);

	if (mptcp_register_scheduler(&mptcp_sched_ratio))
		return -1;
    
    printk("[moinak]ratio scheduler init. with params: num_segments: %u, cwnd_limited: %u\n", num_segments, cwnd_limited);

	return 0;
}


static void ratio_unregister(void)
{
	mptcp_unregister_scheduler(&mptcp_sched_ratio);
}

module_init(ratio_register);
module_exit(ratio_unregister);

MODULE_AUTHOR("Swetank Kumar Saha");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("RATIO MPTCP");
MODULE_VERSION("0.02");

