import React, { useState, useRef, useEffect } from 'react';
const Chatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);
  useEffect(() => { messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' }); }, [messages]);
  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim()) return;
    setMessages(prev => [...prev, { role: 'user', content: inputValue }]);
    const query = inputValue;
    setInputValue('');
    setIsLoading(true);
    try {
      const res = await fetch('https://saimaamjad-reg-chatbot.hf.space/v1/query', {
        method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ query })
      });
      const data = await res.json();
      setMessages(prev => [...prev, { role: 'assistant', content: data.answer, time: data.response_time_ms }]);
    } catch { setMessages(prev => [...prev, { role: 'error', content: 'Connection failed.' }]); }
    finally { setIsLoading(false); }
  };
  const m = typeof window !== 'undefined' && window.innerWidth < 500;
  return (
    <>
      <style>{`@keyframes p{0%,100%{transform:scale(1)}50%{transform:scale(1.08)}}@keyframes s{from{opacity:0;transform:translateY(20px)}to{opacity:1;transform:translateY(0)}}@keyframes d{0%,80%,100%{transform:translateY(0)}40%{transform:translateY(-6px)}}.cb:hover{transform:scale(1.1)!important}.ch:hover{background:rgba(255,255,255,0.35)!important}`}</style>
      {!isOpen&&<button className="cb" onClick={()=>setIsOpen(true)} style={{position:'fixed',bottom:m?15:25,right:m?15:25,width:m?56:64,height:m?56:64,borderRadius:'50%',background:'linear-gradient(135deg,#667eea,#764ba2)',color:'#fff',border:'none',cursor:'pointer',fontSize:m?26:30,boxShadow:'0 6px 25px rgba(102,126,234,0.5)',zIndex:9999,display:'flex',alignItems:'center',justifyContent:'center',transition:'all .3s',animation:'p 2s infinite'}}>💬</button>}
      {isOpen&&<div style={{position:'fixed',bottom:m?0:20,right:m?0:20,left:m?0:'auto',width:m?'100%':400,height:m?'100%':600,maxHeight:m?'100vh':'85vh',background:'#fff',borderRadius:m?0:24,boxShadow:'0 20px 60px rgba(0,0,0,0.25)',zIndex:9999,display:'flex',flexDirection:'column',overflow:'hidden',animation:'s .3s',fontFamily:'-apple-system,BlinkMacSystemFont,Segoe UI,Roboto,sans-serif'}}>
        <div style={{background:'linear-gradient(135deg,#667eea,#764ba2)',color:'#fff',padding:'18px 20px',display:'flex',justifyContent:'space-between',alignItems:'center'}}>
          <div style={{display:'flex',alignItems:'center',gap:12}}><span style={{fontSize:24}}>🤖</span><div><div style={{fontWeight:700,fontSize:17}}>AI Assistant</div><div style={{fontSize:12,opacity:.85}}>Online</div></div></div>
          <div style={{display:'flex',gap:8}}><button className="ch" onClick={()=>setMessages([])} style={{background:'rgba(255,255,255,0.2)',border:'none',color:'#fff',cursor:'pointer',borderRadius:10,padding:'8px 14px',fontSize:13}}>Clear</button><button className="ch" onClick={()=>setIsOpen(false)} style={{background:'rgba(255,255,255,0.2)',border:'none',color:'#fff',fontSize:20,cursor:'pointer',borderRadius:10,width:38,height:38,display:'flex',alignItems:'center',justifyContent:'center'}}>✕</button></div>
        </div>
        <div style={{flex:1,overflowY:'auto',padding:20,background:'linear-gradient(180deg,#f8faff,#f0f4ff)'}}>
          {messages.length===0&&<div style={{textAlign:'center',padding:'50px 20px'}}><div style={{fontSize:55,marginBottom:20}}>📚</div><p style={{color:'#64748b',fontSize:15,lineHeight:1.7}}><strong style={{color:'#334155',fontSize:17}}>Welcome!</strong><br/><br/>Ask me anything about<br/>Physical Humanoid Robotics</p></div>}
          {messages.map((msg,i)=><div key={i} style={{marginBottom:14,display:'flex',justifyContent:msg.role==='user'?'flex-end':'flex-start'}}><div style={{maxWidth:'82%',padding:'14px 18px',fontSize:14,lineHeight:1.6,borderRadius:msg.role==='user'?'20px 20px 6px 20px':'20px 20px 20px 6px',...(msg.role==='user'?{background:'linear-gradient(135deg,#667eea,#764ba2)',color:'#fff'}:msg.role==='error'?{background:'#fee2e2',color:'#dc2626'}:{background:'#fff',color:'#1e293b',boxShadow:'0 2px 12px rgba(0,0,0,0.08)'})}}>{msg.content}{msg.time&&<div style={{fontSize:11,opacity:.7,marginTop:8,textAlign:'right'}}>⚡{msg.time}ms</div>}</div></div>)}
          {isLoading&&<div style={{marginBottom:14}}><div style={{background:'#fff',padding:'18px 22px',borderRadius:'20px 20px 20px 6px',boxShadow:'0 2px 12px rgba(0,0,0,0.08)',display:'inline-block'}}><div style={{display:'flex',gap:6}}>{[0,1,2].map(i=><span key={i} style={{width:10,height:10,background:'#667eea',borderRadius:'50%',animation:'d 1.4s infinite',animationDelay:i*.16+'s'}}/>)}</div></div></div>}
          <div ref={messagesEndRef}/>
        </div>
        <form onSubmit={handleSubmit} style={{padding:m?'14px 16px':'18px 20px',background:'#fff',borderTop:'1px solid #e5e7eb'}}>
          <div style={{display:'flex',gap:12}}>
            <input type="text" value={inputValue} onChange={e=>setInputValue(e.target.value)} placeholder="Type your question..." disabled={isLoading} style={{flex:1,padding:'14px 18px',border:'2px solid #e5e7eb',borderRadius:16,fontSize:15,outline:'none'}}/>
            <button type="submit" disabled={isLoading||!inputValue.trim()} style={{padding:'14px 22px',background:isLoading||!inputValue.trim()?'#cbd5e1':'linear-gradient(135deg,#667eea,#764ba2)',color:'#fff',border:'none',borderRadius:16,cursor:isLoading||!inputValue.trim()?'not-allowed':'pointer',fontSize:18,fontWeight:600}}>{isLoading?'...':'➤'}</button>
          </div>
        </form>
      </div>}
    </>
  );
};
export default Chatbot;
