
// do this before including other stuff like wificlient.h
#include "lwip/tcp_impl.h"

void tcpCleanup()
{
  while(tcp_tw_pcbs!=NULL)
  {
    tcp_abort(tcp_tw_pcbs);
  }
}