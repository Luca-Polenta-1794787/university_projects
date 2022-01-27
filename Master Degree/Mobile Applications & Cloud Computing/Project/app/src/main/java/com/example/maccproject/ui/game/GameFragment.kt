package com.example.maccproject.ui.game

import android.os.Bundle
import android.text.method.ScrollingMovementMethod
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import androidx.fragment.app.Fragment
import androidx.lifecycle.ViewModelProvider
import com.example.maccproject.databinding.FragmentGameBinding

class GameFragment : Fragment() {

    private lateinit var gameViewModel: GameViewModel
    private var _binding: FragmentGameBinding? = null

    // This property is only valid between onCreateView and
    // onDestroyView.
    private val binding get() = _binding!!

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle? ): View {
        gameViewModel = ViewModelProvider(this)[GameViewModel::class.java]

        _binding = FragmentGameBinding.inflate(inflater, container, false)
        val root: View = binding.root

        val textViewTitle: TextView = binding.titleGame
        gameViewModel.title.observe(viewLifecycleOwner, {
            textViewTitle.text = it
        })
        val textViewSubtitle: TextView = binding.explanationGame
        gameViewModel.explanation.observe(viewLifecycleOwner, {
            textViewSubtitle.text = it
        })

        binding.explanationGame.movementMethod = ScrollingMovementMethod()

        setHasOptionsMenu(true)

        return root
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }



}