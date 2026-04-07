
namespace nmxui
{
	
	void GLWindow::setCurrent(void)
	{
		wglMakeCurrent(m_devicecontext, m_glcontext);
	}
	
	void GLWindow::render(void)
	{
		//pseudocode
		//
		//_DWORD *v2; // ebx
		//int *v3; // edi
		//int v4; // eax
		//int v5; // eax
		//_DWORD *v6; // ebx
		//_DWORD *v7; // ebp
		//int *v8; // edi
		//int v9; // eax
		//int v10; // eax
		//_DWORD *v11; // [esp+10h] [ebp-Ch]
		//int *v12; // [esp+14h] [ebp-8h]
		//int *v13; // [esp+14h] [ebp-8h]
		//
		//v2 = (_DWORD *)*((_DWORD *)this + 94);
		//if ( (unsigned int)v2 > *((_DWORD *)this + 95) )
		//	invalid_parameter_noinfo();
		//v3 = (int *)*((_DWORD *)this + 91);
		//v11 = (_DWORD *)*((_DWORD *)this + 95);
		//if ( *((_DWORD *)this + 94) > (unsigned int)v11 )
		//	invalid_parameter_noinfo();
		//v12 = (int *)*((_DWORD *)this + 91);
		//while ( 1 )
		//{
		//	if ( !v3 || v3 != v12 )
		//		invalid_parameter_noinfo();
		//	if ( v2 == v11 )
		//		break;
		//	if ( v3 )
		//	{
		//		v4 = *v3;
		//	}
		//	else
		//	{
		//		invalid_parameter_noinfo();
		//		v4 = 0;
		//	}
		//	if ( (unsigned int)v2 >= *(_DWORD *)(v4 + 16) )
		//		invalid_parameter_noinfo();
		//	(*(void (__thiscall **)(_DWORD, nmxui::GLWindow *))(*(_DWORD *)*v2 + 4))(*v2, this);
		//	if ( v3 )
		//	{
		//		v5 = *v3;
		//	}
		//	else
		//	{
		//		invalid_parameter_noinfo();
		//		v5 = 0;
		//	}
		//	if ( (unsigned int)v2 >= *(_DWORD *)(v5 + 16) )
		//		invalid_parameter_noinfo();
		//	++v2;
		//}
		
		wglMakeCurrent(m_devicecontext, m_glcontext);
		glViewport(0, 0, m_width, m_height);
		//unknown_classfunction();
		SwapBuffers(m_devicecontext);
		
		//pseudocode
		//
		//v6 = (_DWORD *)*((_DWORD *)this + 94);
		//if ( (unsigned int)v6 > *((_DWORD *)this + 95) )
		//	invalid_parameter_noinfo();
		//v7 = (_DWORD *)*((_DWORD *)this + 95);
		//v8 = (int *)*((_DWORD *)this + 91);
		//if ( *((_DWORD *)this + 94) > (unsigned int)v7 )
		//	invalid_parameter_noinfo();
		//v13 = (int *)*((_DWORD *)this + 91);
		//while ( 1 )
		//{
		//	if ( !v8 || v8 != v13 )
		//		invalid_parameter_noinfo();
		//	if ( v6 == v7 )
		//		break;
		//	if ( v8 )
		//	{
		//		v9 = *v8;
		//	}
		//	else
		//	{
		//		invalid_parameter_noinfo();
		//		v9 = 0;
		//	}
		//	if ( (unsigned int)v6 >= *(_DWORD *)(v9 + 16) )
		//		invalid_parameter_noinfo();
		//	(*(void (__thiscall **)(_DWORD, nmxui::GLWindow *))(*(_DWORD *)*v6 + 8))(*v6, this);
		//	if ( v8 )
		//	{
		//		v10 = *v8;
		//	}
		//	else
		//	{
		//		invalid_parameter_noinfo();
		//		v10 = 0;
		//	}
		//	if ( (unsigned int)v6 >= *(_DWORD *)(v10 + 16) )
		//		invalid_parameter_noinfo();
		//	++v6;
		//}
	}
	
	void GLWindow::onPaint(wxPaintEvent& paintevent)
	{
		wxPaintDC paintdevicecontext(this);
		
		//pseudocode
		//
		//v3 = *(void (__thiscall **)(nmxui::GLWindow *))(*(_DWORD *)this + 724);
		//v3(this);
	}
	void GLWindow::onShow(wxShowEvent& showevent)
	{
		wxSizeEvent sizeevent; //unused ?
		GetClientSize(&m_width, &m_height);
		
		//pseudocode
		//
		//v3 = (int *)((char *)this + 400);
		//v6 = 1;
		//v4 = (int *)((char *)this + 396);
		//if ( *((_DWORD *)this + 99) <= 1u )
		//	v4 = &v6;
		//*((_DWORD *)this + 99) = *v4;
		//v6 = 1;
		//v5 = (int *)((char *)this + 400);
		//if ( (unsigned int)*v3 <= 1 )
		//	v5 = &v6;
		//*v3 = *v5;
		
	}
	void GLWindow::onSize(wxSizeEvent& sizeevent)
	{
		GetClientSize(&m_width, &m_height);
		
		//pseudocode
		//
		//v2 = (int *)((char *)this + 400);
		//v3 = (int *)((char *)this + 396);
		//
		//v5 = 1;
		//v4 = v3;
		//if ( (unsigned int)*v3 <= 1 )
		//	v4 = &v5;
		//*v3 = *v4;
		//v5 = 1;
		//if ( (unsigned int)*v2 <= 1 )
		//	*v2 = v5;
		//else
		//	*v2 = *v2;
	}
	
} //namespace nmxui